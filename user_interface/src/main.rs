mod ui_thread;
mod read_csv;
use std::{thread, time::Duration};

use read_csv::*;
use ui_thread::*;

fn main() {
    // Read your CSV data into vectors
    let accel = read_csv("user_interface/sensor_data/Accelerometer.csv").unwrap();
    let gyro = read_csv("user_interface/sensor_data/Gyroscope.csv").unwrap();
    let zipped: Vec<((f32, f32, f32, f32), (f32, f32, f32, f32))> =
        accel.into_iter().zip(gyro.into_iter()).collect();

    // Start the UI. ui_start returns a Sender for sensor updates.
    let (ui_tx, my_ui )= ui_start();

    // Spawn a thread to send sensor data to the UI.
    let tt = thread::spawn(move || {
        thread::sleep(Duration::from_secs(5));
        for data in zipped {

            let time = (data.0).0;

            let message = (
                0.0, // pitch (or computed value)
                0.0, // roll
                0.0, // throttle
                (data.1).1, // gyro x
                (data.1).2, // gyro y
                (data.1).3, // gyro z
                (data.0).1, // accel x
                (data.0).2, // accel y
                (data.0).3, // accel z
                time
            );

            // Send the sensor update to the UI.
            ui_tx.send(message).unwrap();

            println!("Sleeping for 10ms, data has been sent: {:?}",ui_tx.send(message));
        }
    });

    run(my_ui);

    tt.join();
}