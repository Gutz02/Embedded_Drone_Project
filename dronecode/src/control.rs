use heapless::Vec;
use crate::filtering::yaw_sys::YawSys;
use crate::yaw_pitch_roll::YawPitchRoll;
use tudelft_quadrupel::barometer::read_pressure;
use tudelft_quadrupel::battery::read_battery;
use tudelft_quadrupel::fixed::types::extra::U7;
use tudelft_quadrupel::fixed::FixedI16;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::mpu::{enable_dmp, read_dmp_bytes, read_raw};
use tudelft_quadrupel::mpu::structs::{Accel, Gyro};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::send_bytes;
use communication::dep_messages::{set_feedback, set_logging, ControlPacket, JoystickCommand, KalmanPacket, MessageType, State, TrimCommand, ValidFlash, ResetYaw, Bluetooth};
use tudelft_quadrupel::flash::{flash_chip_erase, flash_write_bytes, flash_read_bytes};
use tudelft_quadrupel::nrf51_pac::{Peripherals, RADIO};
use communication::initialize::{init_hfclk, init_rx_mode, init_tx_mode, radio_initialize, transmit_packet};
use crate::filtering::filter_util::*;
use crate::filtering::{butter_filter::ButterStack, kalman_filter::Kalman, atan_lut::Lut};
use crate::fsm::{CurrentAction, DroneFSM};
use crate::util::*;

static mut RX_BUFFER_DRONE: crate::tx_rx_pc_side::AlignedPacketBuf = crate::tx_rx_pc_side::AlignedPacketBuf([0; 129]);
pub const FREQUENCY : u64 = 250;
pub const DTIME : f32 = 1.0/(FREQUENCY as f32);

pub fn control_loop(logging : bool) -> ! {
    set_tick_frequency(FREQUENCY);
    let mut fsm = DroneFSM::init();
    let mut panic_cnt: u8 = 0;
    let mut battery = false;
    let mut bluetooth = false;

    // Storing calibrated values
    let calibrated_accel : AccelFiltered = AccelFiltered::new();
    let calibrated_gyro : GyroFiltered   = GyroFiltered::new();
    // For raw debugging
    let mut offset_accel_raw : Accel = Accel{x: 0, y: 0, z: 0};
    let mut offset_gyro_raw : Gyro = Gyro{x: 0, y: 0, z: 0};

    let mut calibrated_pitch = 0.0;
    let mut calibrated_roll = 0.0;
    let mut calibrated_yaw = 0.0;
    let calibrated_pressure= 101325.0;

    // Storing runtime command variables
    let mut joystick: JoystickCommand = JoystickCommand::new(0, 0, 0, 0);

    let mut full_control_variables : ((f32,f32,f32),(f32,f32,f32),f32) = ((0.0,0.0,0.0),(0.0,0.0,0.0),0.0);
    let mut yaw_proportional : f32 =  0.0;

    // Storing runtime drone variables
    let mut log_motors: [u16;4] = [0;4];

    // Store latest ypr reading
    let mut last_pitch = 0.0;
    let mut last_roll = 0.0;
    let mut last_yaw = 0.0;
    let mut err_cnt: u16 = 1;
    let mut prev_err_cnt: u16 = 1;

    let roll_cumul_err = 0.0;
    let pitch_cumul_err = 0.0;

    let mut prev_err_roll:f32 = 0.0;
    let mut prev_err_pitch:f32 = 0.0;

    // Height control variables
    let mut prev_time: Instant = Instant::now(); // it's in ns
    let vertical_velo : Fix16_7 = Fix16_7::from_num(0.0);
    let altitude_est : Fix16_7 = Fix16_7::from_num(0.0);

    // Filtering results :)
    let mut dmp_reading : bool = false;
    let mut is_calibrated = false;
    let mut filtered_signal : (Accel,Gyro);
    // Change here for changing order butterworth 5/5
    let a : [Fix16_7 ; 2] =  [Fix16_7::from_num(1.0), Fix16_7::from_num(-0.50952545)];
    let b : [Fix16_7 ; 2] =  [Fix16_7::from_num(0.24523728), Fix16_7::from_num(0.24523728)];
    let mut accel_filtered : AccelFiltered  = AccelFiltered::new();
    let mut gyro_filtered  : GyroFiltered   = GyroFiltered::new();
    let mut filtering_stack: ButterStack = ButterStack::new(a, b);

    // Kalman Filter
    let lookup_table : Lut = Lut::new();
    let mut yaw_inst     : YawSys = YawSys::new(DTIME);
    let mut kalman_filter = Kalman::new(lookup_table);

    // For logging
    let mut read_address: u32 = 0x000000;
    let mut start_address: u32 = 0x000000;
    const MAX_ADDRESS: u32 = 0x01FFFF;
    flash_chip_erase().expect("Expected erased memory");
    let mut flash_memory: bool = true;

    // DMP angles
    let mut pitch= 0.0;
    let mut roll= 0.0;
    let mut yaw= 0.0;

    // Trimming values
    let mut throttle_trim = 0;
    let mut roll_trim = 0;
    let mut pitch_trim = 0;
    let mut yaw_trim = 0;

    // Kalman angles
    let mut kalman_output : AngleRaw = AngleRaw::new(None,None,None);
    let yaw_rate = 0.0;

    // For yaw variable k
    let mut yaw_k = 0;

    enable_dmp();

    let mut nrf51_peripherals = unsafe {Peripherals::steal()};
    let buffer = unsafe { &mut RX_BUFFER_DRONE.0 };

    init_hfclk(&mut nrf51_peripherals.CLOCK);
    radio_initialize(&mut nrf51_peripherals.RADIO);

    for i in 0.. {


        // currently unused
        // le#[inline(never)]
        let battery_voltage = read_battery();
        // If we use the drone, set battery = true
        // Current battery boolean: false
        if (battery && battery_voltage < 11) {
            fsm.set_panic(&mut is_calibrated);
        }
        let state = fsm.get_state();
        let pressure = read_pressure();

        if bluetooth {
            buffer.fill(0);
            init_rx_mode(&mut nrf51_peripherals.RADIO);
        }

        let decoded = fsm.action(&mut is_calibrated, bluetooth, &mut nrf51_peripherals.RADIO, buffer);
        let (accel, gyro) = read_imu(&offset_accel_raw, &offset_gyro_raw, fsm.get_state(), &mut is_calibrated);

        // Try to read height
        // let current_time = Instant::now();
        // let dt: f64;
        // if prev_time.ns_since_start() > 0  {
        //     dt = current_time.duration_since(prev_time).as_millis_f64()
        // } else {
        //     dt = 0.0
        // }
        // prev_time = current_time;

        // Uncomment the following for height control:
        // let new_vel;
        // let new_altitude;
        
        if !dmp_reading{
            last_yaw = yaw;
            // We start by filtering the IMU readings
            (accel_filtered, gyro_filtered) = filtering_stack.filter_raw(accel, gyro);

            // Uncomment the following for height control:
            // (new_vel, new_altitude) = get_altitude(dt, pressure, calibrated_pressure as u32, accel_filtered.z.to_bits(), vertical_velo, altitude_est);
            // vertical_velo = new_vel;
            // altitude_est = new_altitude;

            // Calculate angles from accelerometer data (measurement)
            let accel_squared : [Fix32_14;3] = kalman_filter.square_accel(&accel_filtered);
            let angle_roll: Fix16_7 = kalman_filter.angle_roll(&accel_filtered, &accel_squared);
            let angle_pitch: Fix16_7 = kalman_filter.angle_pitch(&accel_filtered, &accel_squared);
            let roll_rate : Fix16_7 = gyro_filtered.x;
            let pitch_rate: Fix16_7 = gyro_filtered.y;

            // Retrieve the current state estimates and uncertainties from the Kalman filter struct
            let roll_uncertainty:Fix16_7 = kalman_filter.k_uncertainty_roll;
            let pitch_uncertainty:Fix16_7 = kalman_filter.k_uncertainty_pitch;
            last_roll  = kalman_output.roll.to_num::<f32>();
            last_pitch = kalman_output.pitch.to_num::<f32>();

            kalman_output = kalman_filter.kalman_predict(
                kalman_output.roll, roll_uncertainty, roll_rate, angle_roll,
                kalman_output.pitch, pitch_uncertainty, pitch_rate, angle_pitch
            );
            yaw = yaw_inst.decision(gyro.z);
            kalman_output.yaw = Fix16_7::from_num(yaw);
            
            roll = kalman_output.roll.to_num::<f32>();
            pitch = kalman_output.pitch.to_num::<f32>();
        }else if dmp_reading {
            // Uncomment the following for height control:
            // WARNING: FOR HEIGHT CONTROL:
            // THIS NOW USES UNFILTERED ACC VALUES
            // (new_vel, new_altitude) = get_altitude(dt, pressure, calibrated_pressure as u32, accel.z, vertical_velo, altitude_est);
            // vertical_velo = new_vel;
            // altitude_est = new_altitude;
            match read_dmp_bytes() {
                Ok(quaternion) => {
                    /// update latest ypr reading
                    let ypr = YawPitchRoll::from(quaternion);
                    last_yaw = yaw;
                    last_roll = roll;
                    last_pitch = pitch;
                    pitch = ypr.pitch + 0.025*pitch_trim as f32;
                    roll = ypr.roll + 0.025*roll_trim as f32;
                    yaw = ypr.yaw + yaw_trim as f32;
                    prev_err_cnt = err_cnt;
                    err_cnt = 1;
                },
                Err(_) => {
                    err_cnt = err_cnt + 1;
                }
            }
        }

        let state = fsm.get_state();

        /// Do stuff based on msg received
        match &decoded {
            MessageType::Dummy => {
                panic_cnt += 1;
                if panic_cnt > 50 {
                    // For panicking when drone disconnects
                    //panic!();
                    fsm.set_panic(&mut is_calibrated);
                    panic_cnt = 0;
                }
            },
            MessageType::JoystickCommand(command) => {
                if JoystickCommand::check_packet(&command) {
                    joystick = command.clone();
                }
                panic_cnt = 0;
            }
            MessageType::ControlPacket(command) => {
                if ControlPacket::check_packet(&command) {
                    full_control_variables = command.full_variables;
                    yaw_k = command.yaw_k;
                    yaw_proportional = command.yaw_proportional;
                    panic_cnt = 0;
                }
            }
            MessageType::KalmanPacket(command) => {
                if KalmanPacket::check_packet(&command){
                    // kalman_filter.k_meas_noise = Fix16_7::from_num(command.k_measurment_noise);
                    // kalman_filter.k_proc_noise = Fix16_7::from_num(command.k_process_noise);
                    // panic_cnt = 0;
                }
            }
            MessageType::FlashMemory(command) => {
                if ValidFlash::check_packet(&command) {
                    flash_memory = true;
                }
                panic_cnt = 0;
            }
            MessageType::TrimDrone(command) => {
                if TrimCommand::check_packet(&command) {
                    throttle_trim = command.throttle_trim;
                    roll_trim = command.roll_trim;
                    pitch_trim = command.pitch_trim;
                    yaw_trim = command.yaw_trim;
                    yaw_inst.set_trim(yaw_trim);
                    kalman_filter.set_trim(command.roll_trim, command.pitch_trim);
                }
                panic_cnt = 0;
            }
            MessageType::ResetYaw(command) => {
                if ResetYaw::check_packet(command){
                    yaw_inst.reset();
                }
            }
            MessageType::Bluetooth(command) => {
                bluetooth = command.is_initialized;
                send(MessageType::Bluetooth(Bluetooth::new(bluetooth)), &mut nrf51_peripherals.RADIO, false);
            }
            _ => {
                panic_cnt = 0;
            }
        }

        // Do stuff based on drone state
        match state{
            State::Safe => {set_motors([0,0,0,0]);}
            State::Calibration => {
                offset_accel_raw = accel.clone();
                offset_gyro_raw =  gyro.clone();
                calibrated_pitch = pitch;
                calibrated_roll = roll;
                calibrated_yaw = yaw;
                yaw_inst.reset();
                kalman_filter.reset();
                kalman_filter.set_offset();
                is_calibrated = true;
                if !dmp_reading{
                    kalman_filter.set_offset();
                }
                fsm.transition(State::Safe, &mut is_calibrated);
            }
            State::Manual => {
                log_motors = joystick_set_motors(&joystick, throttle_trim, yaw_k);
                set_motors(log_motors);
            }
            State::YawControl => {
                log_motors = yaw_set_motors(&joystick, yaw, last_yaw, yaw_proportional, yaw_rate, throttle_trim, yaw_k);
                set_motors(log_motors);
            }
            State::FullControl => {
                dmp_reading = true;
                log_motors = full_set_motors(
                    &joystick, yaw, last_yaw, full_control_variables, prev_err_cnt,
                        roll, last_roll, &mut prev_err_roll, pitch
                    , last_pitch, &mut prev_err_pitch, roll_cumul_err, pitch_cumul_err,throttle_trim, yaw_k);
                set_motors(log_motors);
            }
            State::FullControlRaw => {
                dmp_reading = false;
                log_motors = full_set_motors_raw(
                    &joystick, yaw, last_yaw, full_control_variables, prev_err_cnt,
                        roll, last_roll, &mut prev_err_roll, pitch
                    , last_pitch, &mut prev_err_pitch, roll_cumul_err, pitch_cumul_err,throttle_trim, yaw_k);
                set_motors(log_motors);
            }
            _ => {}
        }

        /// For logging
        if logging{
            send_match(decoded, &mut nrf51_peripherals.RADIO, bluetooth);
            if i % 10 == 0 {
                if dmp_reading {
                    send(MessageType::FeedBack(
                        set_feedback(
                            accel.x,
                            accel.y,
                            accel.z,
                            gyro.x,
                            gyro.y,
                            gyro.z,
                            (pitch,roll,yaw),
                            fsm.get_state(),
                            log_motors[0],
                            log_motors[1],
                            log_motors[2],
                            log_motors[3],
                            full_control_variables,
                            yaw_proportional,
                            kalman_filter.packet(),
                            pressure,
                            battery_voltage as u32,
                            altitude_est.to_num::<i16>()
                        )
                    ), &mut nrf51_peripherals.RADIO, bluetooth);
                } else {
                    send(MessageType::FeedBack(
                        set_feedback(
                           (accel_filtered.x).to_num::<i16>(),
                           (accel_filtered.y).to_num::<i16>(),
                           (accel_filtered.z).to_num::<i16>(),
                            (gyro_filtered.x).to_num::<i16>(),
                            (gyro_filtered.y).to_num::<i16>(),
                            gyro.z,
                    kalman_output.to_f32(yaw),
                            fsm.get_state(),
                            log_motors[0],
                            log_motors[1],
                            log_motors[2],
                            log_motors[3],
                            full_control_variables,
                            yaw_proportional,
                            kalman_filter.packet(),
                            pressure,
                            battery_voltage as u32,
                            altitude_est.to_num::<i16>()
                        )
                    ), &mut nrf51_peripherals.RADIO, bluetooth);
                }
            }

            /// Logging drone side
            if i % 1 == 0 {
                let logging = MessageType::LoggingDrone(
                    set_logging(
                        (accel_filtered.x).to_num::<i16>(),
                        (accel_filtered.y).to_num::<i16>(),
                        (accel_filtered.z).to_num::<i16>(),
                         (gyro_filtered.x).to_num::<i16>(),
                         (gyro_filtered.y).to_num::<i16>(),
                            gyro.z,
                        log_motors[0],
                        log_motors[1],
                        log_motors[2],
                        log_motors[3],
                    )
                );
                let logging_bytes = postcard::to_vec::<_, 64>(&logging).unwrap();
                let length = logging_bytes.len() as u8;

                let mut data: Vec<u8, 64> = Vec::new();
                data.push(length).unwrap();
                data.extend_from_slice(&logging_bytes).unwrap();

                // Write to flash
                match flash_write_bytes(start_address, &data) {
                    Ok(_) => {
                        start_address += data.len() as u32;
                        if start_address > MAX_ADDRESS {
                            flash_chip_erase().expect("Expected erased memory");
                            start_address = 0x000000;
                            read_address = 0x000000;
                        }
                    }
                    Err(_) => {
                        flash_chip_erase().expect("Expected erased memory");
                        start_address = 0x000000;
                        read_address = 0x000000;
                    }
                }
            }

            // Flush the data on the flash
            if flash_memory {
                while read_address < start_address {
                    let mut buffer = [0u8; 64];

                    match flash_read_bytes(read_address, &mut buffer) {
                        Ok(_) => {
                            let len = buffer[0] as usize;
                            if len > 64 {
                                // Added this because otherwise drone panics
                                flash_chip_erase().expect("len cannot be bigger than 64");
                                start_address = 0x000000;
                                read_address = 0x000000;
                            } else {
                                let record_data = &buffer[0..(len + 1)];
                                send_bytes(record_data);
                                read_address += record_data.len() as u32;
                            }
                        }
                        Err(_) => {}
                    }
                }
                flash_chip_erase().expect("Expected erased memory after being done flushing");
                start_address = 0x000000;
                read_address = 0x000000;
                flash_memory = false;
            }
        }
        wait_for_next_tick();
    }
    panic!();
}

pub fn read_imu( accel_raw_offset: &Accel, gyro_raw_offset: &Gyro, state: State, is_calibrated: &mut bool) -> (Accel, Gyro) {
    if let State::Calibration = state{
        let (a, g) = read_raw().unwrap();
        *is_calibrated = true;
        return (
            a, g
        );
    }

    let (accel, gyro) = read_raw().unwrap();
    (
        accel_diff(&accel, accel_raw_offset),
        gyro_diff(&gyro,   gyro_raw_offset),
    )
}



pub fn send_match(decoded: MessageType, radio: &mut RADIO, bluetooth: bool) {
    match decoded {
        MessageType::Dummy => {send(MessageType::Dummy, radio, bluetooth)},
        MessageType::ChangeMode(cmd) => {send(MessageType::ChangeMode(cmd), radio, bluetooth)},
        MessageType::JoystickCommand(cmd) => {send(MessageType::JoystickCommand(cmd), radio, bluetooth)}
        MessageType::KeyBoardCommand(cmd) => {send(MessageType::KeyBoardCommand(cmd), radio, bluetooth)},
        MessageType::ControlPacket(cmd) => {send(MessageType::ControlPacket(cmd), radio, bluetooth)},
        MessageType::FeedBack(cmd) => {send(MessageType::FeedBack(cmd), radio, bluetooth)}
        MessageType::LoggingDrone(cmd) => {send(MessageType::LoggingDrone(cmd), radio, bluetooth)}
        MessageType::FlashMemory(cmd) => {send(MessageType::FlashMemory(cmd), radio, bluetooth)}
        MessageType::TrimDrone(cmd) => {send(MessageType::TrimDrone(cmd), radio, bluetooth)}
        MessageType::Bluetooth(cmd) => {send(MessageType::Bluetooth(cmd), radio, bluetooth)}
        _ => {}
    }
}

pub(crate) fn send(msg: MessageType, radio: &mut RADIO, bluetooth: bool) {
    let mut serialize = postcard::to_vec::<_, 128>(&msg).unwrap();
    let len = serialize.len();
    let _ = serialize.insert(0, len as u8); // this won't panic
    if bluetooth {
        init_tx_mode(radio);
        transmit_packet(radio, &serialize);
    }
    else {
        send_bytes(&serialize);
    }
}