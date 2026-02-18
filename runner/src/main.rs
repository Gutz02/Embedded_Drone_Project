use std::env::args;
use std::fs::{File, OpenOptions};
use std::path::PathBuf;
use std::process::{exit, Command};
use std::time::Duration;
use std::thread::{self};
use std::sync::{Mutex, OnceLock};
use std::sync::Arc;
use std::io::{Write, Read};
use communication::dep_messages::*;
use tudelft_serial_upload::{upload_file_or_stop, PortSelector};
use tudelft_serial_upload::serial2::SerialPort;
use controller::GamepadController;
use chrono::Utc;
use fixed::{FixedI16, types::extra::{U7,U14}, FixedI32};

// For User interface
use user_interface::ui_thread::*;
use std::sync::mpsc::{Sender};
use communication::dep_messages::State::{Panic, YawControl};

const CONTROLLER_PERIOD: u64 = 50;
// For the filename in a global variable to be able to write to it
static FILENAME: OnceLock<String> = OnceLock::new();
static FILENAME_LOGGING: OnceLock<String> = OnceLock::new();
const WANT_CSV: bool = true;
const BLUETOOTH_DRONE_PART: bool = false;
const WANT_CSV_LOGGING: bool = false;

fn main() {
    // For the logging feedback data from drone in an .csv
    // TIP: Download a plugin for csv files for rainbow colored
    if WANT_CSV {
        let nowtime = Utc::now();
        let filename = format!("logging_data_{}.csv", nowtime.format("%Y-%m-%d_%H-%M-%S"));
        FILENAME.set(filename.clone()).expect("Filename setting has failed");
        let mut file_csv = File::create(filename).expect("Creating file has failed");

        // Headers for the logging of drone (based on get_feedback())
        writeln!(file_csv, "gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,count,state,m_1,m_2,m_3,m_4,k_p").expect("Writing csv header has failed");
        dbg!("THE FILE IS CREATED");
    } else {
        dbg!("NO FILE CREATED (bool = false)");
    }

    // For the logging feedback data from drone flash in an .csv
    if (WANT_CSV_LOGGING) {
        let nowtime = Utc::now();
        let filename = format!("drone_logging_data_{}.csv", nowtime.format("%Y-%m-%d_%H-%M-%S"));
        FILENAME_LOGGING.set(filename.clone()).expect("Filename setting has failed");
        let mut file_csv = File::create(filename).expect("Creating file has failed");

        // Headers for the logging of drone (based on get_logging())
        writeln!(file_csv, "gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,count,m_1,m_2,m_3,m_4").expect("Writing csv header has failed");
        dbg!("THE FILE IS CREATED FOR DRONE LOGGING");
    } else {
        dbg!("NO FILE CREATED FOR DRONE LOGGING (bool = false)");
    }

    // UART configuration
    let file = args().nth(1);
    let port = upload_file_or_stop(PortSelector::AutoManufacturer, file);
    let serial = Arc::new(Mutex::new(SerialPort::open(port, 115200).unwrap()));
    {
        let mut s_p_g = serial.lock().unwrap();
        s_p_g.set_read_timeout(Duration::from_secs(1)).unwrap();
        s_p_g.set_write_timeout(Duration::from_secs(1)).unwrap();
    }
    let (tx_drone, tx_controller, my_ui)= ui_start(serial.clone());

    //Controller Thread
    let _ = controller_thread(tx_controller, &serial);

    // Terminal output Thread?
    let _ = display_drone_messages(&serial, tx_drone);

    ui_run(my_ui);
}

fn motor_filter(throttle: f32) -> i16 {
    // Pre-check if throttle must be positive
    if throttle < 0.0 {
        println!("{throttle}");
        panic!("ERROR: Throttle cannot be negative");
    }

    // if less than 3% then this is deadzone
    if throttle < 3.0 {
        return 0;
    }

    // Dead zone 0-3%
    // rest 97%
    // 3-10% range 180-250 -> 10 change / %
    // 10-70% range 250-400 -> 2.5 change / %
    // 70-100% range 400-600 -> 6.7 change / %
    if throttle < 10.0 {
        (180.0 + ((throttle - 3.0) * 10.0)) as i16
    } else if throttle < 70.0 {
        (250.0 + ((throttle - 10.0) * 2.5)) as i16
    } else if throttle <= 100.0 {
        (400.0 + ((throttle - 70.0) * 6.6)) as i16
    }
    // Pre-check if throttle is above 100%
    else {
        panic!("ERROR: Throttle cannot be above 100%");
    }
}

fn joystick_filter(rate: f32) -> i16 {
    // if less than 2% then this is deadzone
    if rate.abs() < 2.0 {
        return 0;
    }

    // rest 98%
    // 2-52% range 0-40 -> 0.8 change / %
    // 52-80% range 40-70 -> 1.071... change / %
    // 80-100% range 70-100 -> 1.5 change / %
    if rate.abs() < 52.0 {
        if rate >= 0.0 {
            ((rate - 2.0) * 0.8) as i16
        } else {
            ((rate + 2.0) * 0.8) as i16
        }

    } else if rate.abs() < 80.0 {
        if rate >= 0.0 {
            (40.0 + ((rate - 52.0) * (30.0/28.0))) as i16
        } else {
            (-40.0 + ((rate + 52.0) * (30.0/28.0))) as i16
        }
    } else if rate.abs() <= 100.0 {
        if rate >= 0.0 {
            (70.0 + ((rate - 80.0) * 1.5)) as i16
        } else {
            (-70.0 + ((rate + 80.0) * 1.5)) as i16
        }
    }
    // Pre-check if throttle is above 100%
    else {
        panic!("ERROR: Throttle cannot be above 100%");
    }

}


fn controller_thread(tx_controller : Sender<(f32,f32,f32,f32,bool)>, serial : &Arc<Mutex<SerialPort>>) {
    let controller_serial = serial.clone();
    thread::spawn(move || {
        let mut controller = GamepadController::new().expect("Gamepad init failed");
        // let mut last_dir: Option<(i16, i16)>  = None;
        // let mut last_panic: Option<bool> = None;
        // let mut last_dir_z: Option<i16> = None;
        // let mut last_rotorspeed: Option<i16> = None;

        // let mut last_status = None;
        // For now rate limiter so no panics
        // TODO: fixing that this rate limiter is not needed
        let mut number = 1;

        // let mut start_time = Utc::now().time();
        // dbg!(start_time);
        loop {
            controller.update();
            // let current_dir = controller.get_direction();
            // let panic_b = controller.get_panic_b();
            // let current_dir_z = controller.get_z_direction();
            // let current_rotorspeed = controller.get_rotorspeed();

            // let current_status = controller.get_status();
            number = number + 1;

            if number % 1 == 0 {
                if let Some((button, x, y, z, rpm, initialized)) = controller.get_status() {
                    let msg: MessageType;

                    if button {
                        //msg = MessageType::Dummy; //weird behaviour when ChangeMode is Panic (probably due to disconnected controller)
                        if !BLUETOOTH_DRONE_PART {
                            msg = MessageType::ChangeMode(ChangeMode::new(Panic));
                        } else {
                            msg = MessageType::Dummy;
                        }
                    }
                    else {
                        // Filters smoothens input before passing the values to the drone
                        msg = MessageType::JoystickCommand(JoystickCommand::new(
                            motor_filter(rpm),
                            joystick_filter(y),
                            joystick_filter(z),
                            joystick_filter(x)));
                    }
                    let encode_msg = postcard::to_vec::<_, 128>(&msg).unwrap();

                    // println!("Joystick input: {:?}", msg);
                    // println!("Joystick encoded: {:?}", encode_msg);

                    // let bit_string = encode_msg.iter().map(|b| format!("{:08b}", b)).collect::<String>();
                    // println!("Bit sequence sent: {:?}", bit_string);

                    let guard = controller_serial.lock().unwrap();
                    guard.write_all(&encode_msg).unwrap();
                    guard.flush().unwrap();

                    tx_controller.send((y as f32, x as f32, z as f32, rpm as f32, initialized)).expect("Failed to send controller Values to UI");
                    // let end_time = Utc::now().time();
                    // let diff = end_time - start_time;
                    // println!("Total time taken to run is {}", diff.num_milliseconds());
                    // start_time = end_time;

                }
                // show if something is wrong with current_status
                else { println!("CURRENT STATUS (PARSING) HAS SOME KIND OF ERROR"); }

                // last_status = current_status;
            }

            thread::sleep(Duration::from_millis(CONTROLLER_PERIOD));
        }
    });
}

// fn enter_panic_state(serial : &Arc<Mutex<SerialPort>>){
//     let controller_serial = serial.clone();
//     let msg = MessageType::ChangeMode(ChangeMode::new(Panic));
//     let encode_msg = postcard::to_vec::<_, 64>(&msg).unwrap();
//
//     println!("Panic state set");
//     println!("Encoded message: {:?}", encode_msg);
//
//     let bit_string = encode_msg.iter().map(|b| format!("{:08b}", b)).collect::<String>();
//
//     println!("Bit sequence sent: {:?}", bit_string);
//
//     let mut guard = controller_serial.lock().unwrap();
//     guard.write_all(&encode_msg).unwrap();
//     guard.flush().unwrap();
// }

fn write_to_csv(cmd : &FeedBack, count : &f32) {
    // Get everything you need, the file + contents
    let filename = FILENAME.get().expect("Filename failed to get");
    let mut file = OpenOptions::new().append(true).open(filename).expect("Opening file has failed");
    let (gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z,
        p_r, count_value, state, m_1, m_2, m_3, m_4, full_control,yaw_variable,k_var,pressure,battery_voltage,alt) = get_feedback(cmd, count);
    // Write to it
    writeln!(file, "{},{},{},{},{},{},{},{},{},{},{},{}", gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z,
             count_value, state as u8, m_1, m_2, m_3, m_4).expect("Failed to write data");
}

fn write_to_csv_drone(cmd : &Logging, count : &f32) {
    // Get everything you need, the file + contents
    let filename = FILENAME_LOGGING.get().expect("Filename failed to get");
    let mut file = OpenOptions::new().append(true).open(filename).expect("Opening file has failed");
    let (gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z,
        count_value, m_1, m_2, m_3, m_4) = get_logging(cmd, count);

    // Write to it
    writeln!(file, "{},{},{},{},{},{},{},{},{},{},{}", gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, count_value, m_1, m_2, m_3, m_4).expect("Failed to write data");
}

fn display_drone_messages(
    serial : &Arc<Mutex<SerialPort>>,
    tx_drone : Sender<((i16,i16,i16,i16,i16,i16,(f32,f32,f32),f32,State,u16,u16,u16,u16,((f32,f32,f32),(f32,f32,f32),f32),f32,(f32,f32),u32,u32,i16))>
) {
    let display_serial = serial.clone();
    thread::spawn(move || {
        let mut buf = [0u8; 255];
        let mut accum: Vec<u8> = Vec::new();
        let mut count: f32 = 0.0;

        // let mut panic_cnt: u8 = 0;
        // let mut zero_ok: u8 = 0;

        loop {
            {
                let guard = display_serial.lock().unwrap();
                match guard.read(&mut buf) {
                    Ok(num) => {
                        //println!("read: {}", zero_ok);
                        if !buf.iter().all(|&x| x == 0) {
                            if num > 0 {
                                accum.extend_from_slice(&buf[..num]);

                                loop {
                                    if accum.len() < 1 {
                                        // we need at least 1 byte for the length
                                        break;
                                    }
                                    let length = accum[0] as usize;
                                    if accum.len() < 1 + length {
                                        // not enough data yet
                                        break;
                                    }

                                    let message_bytes = &accum[1..1 + length];
                                    // println!("{:?}",message_bytes);
                                    match postcard::from_bytes::<MessageType>(message_bytes) {
                                        Ok(decoded) => {
                                            match decoded {
                                                MessageType::Dummy => {}
                                                MessageType::ChangeMode(cmd) => {
                                                    println!("Data sent back: {:?}", cmd);
                                                }
                                                MessageType::JoystickCommand(cmd) => {
                                                    println!("Data sent back:  {:?}", cmd);
                                                }
                                                MessageType::KeyBoardCommand(cmd) => {
                                                    println!("Data sent back:  {:?}", cmd);
                                                }
                                                MessageType::ControlPacket(cmd) => {
                                                    println!("Data sent back: {:?}", cmd);
                                                }

                                                MessageType::FeedBack(cmd) => {
                                                    if FeedBack::check_packet(&cmd){
                                                        _ = tx_drone.send(
                                                            get_feedback(&cmd, &count)
                                                        );
                                                        if WANT_CSV {
                                                            write_to_csv(&cmd, &count);
                                                        }
                                                        count = count + 0.04;
                                                    }
                                                    println!("Data sent back: {:?}", cmd);
                                                }
                                                MessageType::DMPAngles(cmd) => {
                                                    println!("Angles: {:?}", cmd);
                                                }
                                                MessageType::LoggingDrone(cmd) => {
                                                    println!("Data sent back: {:?}", cmd);
                                                    if WANT_CSV_LOGGING {
                                                        let good_count = count - 0.04;
                                                        write_to_csv_drone(&cmd, &good_count);
                                                    }
                                                }
                                                MessageType::TrimDrone(cmd) => {
                                                    println!("Data sent back: {:?}", cmd)
                                                }
                                                MessageType::Bluetooth(cmd) => {
                                                    println!("Data sent back: {:?}", cmd)
                                                }
                                                MessageType::AdjustedYaw(cmd) => {
                                                    println!("Adjusted YAW: {:?}", cmd);
                                                }
                                                _ => {}
                                            }
                                        }
                                        Err(e) => {
                                            eprintln!("Failed to deserialize: {:?}", e);
                                        }
                                    }
                                    accum.drain(0..1+length);
                                    // panic_cnt = 0;
                                    // zero_ok = 0;
                                }
                            }
                        }
                    },
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {continue},
                    Err(e) => {
                        // enter_panic_state(&display_serial);
                        eprintln!("Read error: {}, entering panic state", e)
                    },

                }
            }
            thread::sleep(Duration::from_millis(10));
        }
    });
}

fn bytes_to_u128(bytes: &[u8]) -> u128 {
    let mut num = 0u128;
    for (i, &byte) in bytes.iter().enumerate() {
        if i >= 16 { break; }  // u128 can only hold 16 bytes
        num |= (byte as u128) << (120 - 8 * i);
    }
    num
}

fn user_input_thread(s_p: Arc<Mutex<SerialPort>>) {

    thread::sleep(Duration::new(2, 0));
    {

        let a = MessageType::ChangeMode(ChangeMode::new(YawControl));
       //let a = MessageType::Dummy; you shouldn't get anything back when sending this

        let encode_msg = postcard::to_vec::<_, 64>(&a).unwrap();

        println!("User input: {:?}", a);
        println!("User input encoded: {:?}", encode_msg);

        let bit_string = encode_msg.iter().map(|b| format!("{:08b}", b)).collect::<String>();

        println!("Bit sequence sent: {:?}", bit_string);

        let mut s_p_guard = s_p.lock().unwrap();
        s_p_guard.write_all(&encode_msg).unwrap();
        s_p_guard.flush().unwrap();
    }
    
}

#[allow(unused)]
fn start_interface(port: &PathBuf) {
    let mut cmd = Command::new("python");
    cmd
        // there must be a `my_interface.py` file of course
        .arg("my_interface.py")
        // pass the serial port as a command line parameter to the python program
        .arg(port.to_str().unwrap());

    match cmd.output() {
        Err(e) => {
            eprintln!("{}", e);
            exit(1);
        }
        Ok(i) if !i.status.success() => exit(i.status.code().unwrap_or(1)),
        Ok(_) => {}
    }
}
