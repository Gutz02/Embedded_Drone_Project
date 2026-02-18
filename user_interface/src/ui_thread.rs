use std::collections::HashMap;
use eframe::egui::{self, Color32, InputState};
use egui::Key;
use tudelft_serial_upload::color_eyre::owo_colors::OwoColorize;
use std::sync::mpsc;
use std::sync::mpsc::{Receiver, Sender};
use communication::dep_messages::*;
use std::sync::Arc;
use std::sync::{Mutex};
use tudelft_serial_upload::serial2::SerialPort;
use communication::dep_messages::State::{Panic, Safe, YawControl, Manual, Calibration, FullControl, HeightControl, FullControlRaw};
use std::io::{self, Write};
use std::fs;



const MAX_POINTS: usize = 100;
const MAP_LOCATION : &str = "user_interface/src/drone_values.json";

pub struct MyApp {
    pitch_rate: f32,
    roll_rate: f32,
    yaw_rate: f32,
    throttle_rate: f32,


    gyro_x : Vec<[f64; 2]>,
    gyro_y : Vec<[f64; 2]>,
    gyro_z : Vec<[f64; 2]>,
    acce_x : Vec<[f64; 2]>,
    acce_y : Vec<[f64; 2]>,
    acce_z : Vec<[f64; 2]>,
    motors : [u16; 4],
    time   : f64,
    state  : State,
    full_variables : ((f32,f32,f32),(f32,f32,f32),f32),
    full_variables_raw : ((f32,f32,f32),(f32,f32,f32),f32),
    yaw_variables : f32,
    yaw_k_variable : i16,
    kalaman_variables : (f32,f32),
    control_rx_drone: Receiver<((i16,i16,i16,i16,i16,i16,(f32,f32,f32),f32,State,u16,u16,u16,u16,((f32,f32,f32),(f32,f32,f32),f32),f32,(f32,f32),u32,u32,i16))>,
    control_rx_controller : Receiver<(f32, f32, f32, f32, bool)>,
    serial : Arc<Mutex<SerialPort>>,
    pitch_angle : Vec<[f64; 2]>,
    roll_angle : Vec<[f64; 2]>,
    yaw_angle : Vec<[f64; 2]>,
    yaw_angle_circle: f64,
    imu_graph : bool,
    initialized: bool,
    is_calibrated: bool,
    pressure: u32,
    battery_voltage: u32,
    altitude: i16,
    altitude_plot: Vec<[f64; 2]>,

    throttle_trim: i16,
    roll_trim: i16,
    pitch_trim: i16,
    yaw_trim: i16,

    wireless_toggled: bool,

    map_values: HashMap<&'static str, Vec<f32>>
}

impl MyApp {
    fn new(
        rec_drone      : Receiver<((i16,i16,i16,i16,i16,i16,(f32,f32,f32),f32,State,u16,u16,u16,u16,((f32,f32,f32),(f32,f32,f32),f32),f32,(f32,f32),u32,u32,i16))>,
        rec_controller : Receiver<(f32,f32,f32,f32,bool)>,
        serial : Arc<Mutex<SerialPort>>
        ) -> Self {

        let map_values: HashMap<&'static str, Vec<f32>> = HashMap::from([
            ("Yaw Control",       vec![0.0]),
            ("Full Control",      vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ("Full Control Raw",  vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            ("Kalman Filter",     vec![0.0, 0.0, 0.0]),
        ]);

        let mut app = MyApp {
            pitch_rate: 0.0,
            roll_rate: 0.0,
            yaw_rate: 0.0,
            throttle_rate : 0.0,
            gyro_x : Vec::new(),
            gyro_y : Vec::new(),
            gyro_z : Vec::new(),
            acce_x : Vec::new(),
            acce_y : Vec::new(),
            acce_z : Vec::new(),
            time : 0.0,
            full_variables     : ((0.0,0.0,0.0),(0.0,0.0,0.0),0.0),
            full_variables_raw : ((0.0,0.0,0.0),(0.0,0.0,0.0),0.0),
            yaw_variables : 0.0,
            yaw_k_variable: 0,
            kalaman_variables : (16.4,2.0),
            motors : [0,0,0,0],
            state : Safe,
            control_rx_drone : rec_drone,
            control_rx_controller : rec_controller,
            serial,
            roll_angle : Vec::new(),
            pitch_angle : Vec::new(),
            yaw_angle : Vec::new(),
            yaw_angle_circle : 0.0,
            imu_graph : true,
            initialized : false,
            is_calibrated : false,
            pressure : 0,
            battery_voltage: 0,
            altitude: 0,
            altitude_plot: Vec::new(),
            throttle_trim: 0,
            roll_trim: 0,
            pitch_trim: 0,
            yaw_trim: 0,
            wireless_toggled: false,
            map_values: map_values
        };
        app.load_or_init();

        app
    }

    pub fn set_pitch_rate(&mut self, rate : f32){
        self.pitch_rate = rate;
    } 

    pub fn set_roll_rate(&mut self, rate : f32){
        self.roll_rate = rate;
    }

    pub fn set_yaw_rate(&mut self, rate : f32) {
        self.yaw_rate = rate;
    }

    pub fn set_throttle_rate(&mut self, rate : f32){
        self.throttle_rate = rate;
    }

    pub fn set_initialized(&mut self, initialized : bool) {
        self.initialized = initialized;
    }

    pub fn set_gyro_values(&mut self, time: f32, g_x: i16, g_y: i16, g_z: i16) {
        self.gyro_x.push([time as f64, g_x as f64]);
        self.gyro_y.push([time as f64, g_y as f64]);
        self.gyro_z.push([time as f64, g_z as f64]);

    }

    pub fn set_acce_values(&mut self, time: f32, a_x: i16, a_y: i16, a_z: i16) {
        self.acce_x.push([time as f64, a_x as f64]);
        self.acce_y.push([time as f64, a_y as f64]);
        self.acce_z.push([time as f64, a_z as f64]);
    }

    pub fn set_motor_values(&mut self, m_1: u16, m_2 : u16, m_3 : u16, m_4 : u16){
        self.motors[0] = m_1;
        self.motors[1] = m_2;
        self.motors[2] = m_3;
        self.motors[3] = m_4;
    }

    pub fn set_altitude_new(&mut self, time: f32, altitude: i16) {
        self.altitude_plot.push([time as f64, altitude as f64]);
    }


    pub fn set_plot_angles(&mut self, ui: &mut egui::Ui){
        let plot = egui_plot::Plot::new("angle_plot")
            .view_aspect(2.0)
            .height(300.0)
            .x_axis_label("Time")
            .y_axis_label("Angle (degrees)")
            .include_x(self.time)
            .include_y(90)
            .include_y(-90)
            .legend(egui_plot::Legend::default());

        &plot.show(ui, |plot_ui| {

            plot_ui.line(
                egui_plot::Line::new(egui_plot::PlotPoints::from(self.roll_angle.clone()))
                    .name("Roll")
                    .color(egui::Color32::GREEN),
            );

            plot_ui.line(
                egui_plot::Line::new(egui_plot::PlotPoints::from(self.pitch_angle.clone()))
                    .name("Pitch")
                    .color(Color32::RED),
            );

            plot_ui.line(
                egui_plot::Line::new(egui_plot::PlotPoints::from(self.yaw_angle.clone()))
                    .name("Yaw")
                    .color(Color32::BLUE),
            );
        });
    }

    pub fn set_plot_IMU(&mut self, ui: &mut egui::Ui){
        let plot = egui_plot::Plot::new("sensor_plot")
            .view_aspect(2.0)
            .height(300.0)
            .x_axis_label("Time")
            .y_axis_label("Value")
            .include_x(self.time)
            .include_y(-1.5)
            .include_y(1.5)
            .legend(egui_plot::Legend::default());


        &plot.show(ui, |plot_ui| {

            plot_ui.line(
                egui_plot::Line::new(egui_plot::PlotPoints::from(self.gyro_x.clone()))
                    .name("Gyro X")
                    .color(Color32::GREEN),
            );
            plot_ui.line(
                egui_plot::Line::new(egui_plot::PlotPoints::from(self.gyro_y.clone()))
                    .name("Gyro Y")
                    .color(Color32::RED),
            );
            plot_ui.line(
                egui_plot::Line::new(egui_plot::PlotPoints::from(self.gyro_z.clone()))
                    .name("Gyro Z")
                    .color(Color32::BLUE),
            );
            plot_ui.line(
                egui_plot::Line::new(egui_plot::PlotPoints::from(self.acce_x.clone()))
                    .name("Accel X")
                    .color(Color32::YELLOW),
            );
            plot_ui.line(
                egui_plot::Line::new(egui_plot::PlotPoints::from(self.acce_y.clone()))
                    .name("Accel Y")
                    .color(Color32::from_rgb(128, 0, 128)), // Purple
            );
            plot_ui.line(
                egui_plot::Line::new(egui_plot::PlotPoints::from(self.acce_z.clone()))
                    .name("Accel Z")
                    .color(Color32::CYAN),
            );
            plot_ui.line(
                egui_plot::Line::new(egui_plot::PlotPoints::from(self.altitude_plot.clone()))
                    .name("Altitude")
                    .color(Color32::WHITE),
            );
        });
    }

    fn keyboard_commands(&mut self, i : &InputState){
        let mut trimmed = false;

        if i.key_released(Key::Escape) || i.key_released(Key::Space){
            let mut s_p_g = self.serial.lock().expect("Failed to lock serial in UI thread for panic command");
            let msg = MessageType::ChangeMode(ChangeMode::new(Panic));
            println!("Panic button pressed, button sent!");
            let encode_msg = postcard::to_vec::<_, 64>(&msg).unwrap();
            s_p_g.write_all(&encode_msg).unwrap();
            s_p_g.flush().unwrap();
        }

        if i.key_pressed(Key::Num0) {
            self.action_state("Safe");
        }
        if i.key_pressed(Key::Num1) {
            self.action_state("Panic");
        }
        if i.key_pressed(Key::Num2) {
            self.action_state("Manual");
        }
        if i.key_pressed(Key::Num3) {
            self.action_state("Calibrate");
        }
        if i.key_pressed(Key::Num4) {
            self.write_to_file(MAP_LOCATION);
            self.action_state("Yaw Control");
        }
        if i.key_pressed(Key::Num5) {
            self.write_to_file(MAP_LOCATION);
            self.action_state("Full Control");
        }
        if i.key_pressed(Key::Num6) {
            self.write_to_file(MAP_LOCATION);
            self.action_state("Full Control Raw");
        }
        if i.key_pressed(Key::Num7) {
            self.action_state("Height Control");
        }
        if i.key_pressed(Key::Num8) {
            self.action_state("Wireless");
        }


        if i.key_pressed(Key::A) {
            self.throttle_trim += 1;
            trimmed = true;
        }
        if i.key_pressed(Key::Z) {
            self.throttle_trim -= 1;
            trimmed = true;
        }
        if i.key_pressed(Key::ArrowLeft) {
            self.roll_trim -= 1;
            trimmed = true;
        }
        if i.key_pressed(Key::ArrowRight) {
            self.roll_trim += 1;
            trimmed = true;
        }
        if i.key_pressed(Key::ArrowUp) {
            self.pitch_trim += 1;
            trimmed = true;
        }
        if i.key_pressed(Key::ArrowDown) {
            self.pitch_trim -= 1;
            trimmed = true;
        }
        if i.key_pressed(Key::Q) {
            self.yaw_trim -= 1;
            trimmed = true;
        }
        if i.key_pressed(Key::W) {
            self.yaw_trim += 1;
            trimmed = true;
        }

        if (trimmed == true) {
            let mut s_p_g = self.serial.lock().expect("Failed to lock serial in UI thread for trimmed command");
            let msg = MessageType::TrimDrone(TrimCommand::new(self.throttle_trim, self.roll_trim, self.pitch_trim, self.yaw_trim));
            println!("Trimmed called!");
            let encode_msg = postcard::to_vec::<_, 64>(&msg).unwrap();
            s_p_g.write_all(&encode_msg).unwrap();
            s_p_g.flush().unwrap();
            trimmed = false;
        }

        let mut trimmed_PID = false;

        if i.key_pressed(Key::U) {
            if self.state.to_string() == "Yaw Control"{
                self.yaw_variables += 1.0;
            } else if self.state.to_string() == "Full Control Raw" {
                self.full_variables_raw.2 += 1.0;
            }
            else{
                self.full_variables.2 += 1.0;
            }
            trimmed_PID = true;
        }
        if i.key_pressed(Key::J) {
            if self.state.to_string() == "Yaw Control"{
                self.yaw_variables -= 1.0;
            } else if self.state.to_string() == "Full Control Raw" {
                self.full_variables_raw.2 -= 1.0;
            }
            else{
                self.full_variables.2 -= 1.0;
            }
            trimmed_PID = true;
        }

        // Roll pitch P
        if i.key_pressed(Key::I) {
            if self.state.to_string() == "Full Control Raw" {
                self.full_variables_raw.0.0 += 1.0;
                self.full_variables_raw.1.0 += 1.0;
            } else {
                self.full_variables.0.0 += 1.0;
                self.full_variables.1.0 += 1.0;
            }
            trimmed_PID = true;
        }

        if i.key_pressed(Key::K) {
            if self.state.to_string() == "Full Control Raw" {
                self.full_variables_raw.0.0 -= 1.0;
                self.full_variables_raw.1.0 -= 1.0;
            } else {
                self.full_variables.0.0 -= 1.0;
                self.full_variables.1.0 -= 1.0;
            }
            trimmed_PID = true;
        }

        // Roll pitch D
        if i.key_pressed(Key::O) {
            if self.state.to_string() == "Full Control Raw" {
                self.full_variables_raw.0.2 += 1.0;
                self.full_variables_raw.1.2 += 1.0;
            } else {
                self.full_variables.0.2 += 1.0;
                self.full_variables.1.2 += 1.0;
            }
            trimmed_PID = true;
        }
        if i.key_pressed(Key::L) {
            if self.state.to_string() == "Full Control Raw" {
                self.full_variables_raw.0.2 -= 1.0;
                self.full_variables_raw.1.2 -= 1.0;
            } else {
                self.full_variables.0.2 -= 1.0;
                self.full_variables.1.2 -= 1.0;
            }
            trimmed_PID = true;
        }

        if (trimmed_PID == true) {
            let mut s_p_g = self.serial.lock().expect("Failed to lock serial in UI thread for Row");
            let mut msg  = MessageType::ControlPacket(ControlPacket::new(self.yaw_variables, self.yaw_k_variable, self.full_variables));
            let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
            s_p_g.write_all(&encode_msg).unwrap();
            s_p_g.flush().unwrap();
            trimmed_PID = false;
        }
    }

    // Check if it is safe to go to another mode (one of the safety features)
    fn safe_to_mode(&self) -> bool {
        self.initialized && self.throttle_rate == 0.0 && self.pitch_rate == 0.0 && self.roll_rate == 0.0
    }

    fn action_state(&mut self, state: &str) {
        let mut s_p_g = self.serial.lock().expect("Failed to lock serial in UI thread for button command");
        let mut msg :  Option<MessageType> = None;
        match state {
            "Safe" => {
                msg = Some(MessageType::ChangeMode(ChangeMode::new(Safe)));
            },
            "Manual" => {
                if self.safe_to_mode() {
                    msg = Some(MessageType::ChangeMode(ChangeMode::new(Manual)));
                }
            },
            "Panic" => {
                msg = Some(MessageType::ChangeMode(ChangeMode::new(Panic)));
            },
            "Calibrate" => {
                if self.safe_to_mode() {
                    self.yaw_angle_circle = 0.0;
                    msg = Some(MessageType::ChangeMode(ChangeMode::new(Calibration)));
                    self.is_calibrated = true;
                }
            },
            "Yaw Control" => {
                if self.safe_to_mode() {
                    msg = Some(MessageType::ChangeMode(ChangeMode::new(YawControl)));
                }
            },
            "Full Control" => {
                if self.safe_to_mode() {
                    msg = Some(MessageType::ChangeMode(ChangeMode::new(FullControl)));

                }
            }
            "Full Control Raw" => {
                if self.safe_to_mode() {
                    msg = Some(MessageType::ChangeMode(ChangeMode::new(FullControlRaw)));
                }
            }
            "Height Control" => {
                msg = Some(MessageType::ChangeMode(ChangeMode::new(HeightControl)));
            }
            "Wireless" => {
                if self.safe_to_mode() {
                    self.wireless_toggled = !self.wireless_toggled;
                    msg = Some(MessageType::Bluetooth(Bluetooth::new(self.wireless_toggled)));
                    self.state = Safe;
                }
            }
            _ => {}
        }
        if msg.is_some(){
            let encode_msg = postcard::to_vec::<_, 64>(&msg.unwrap()).unwrap();
            s_p_g.write_all(&encode_msg).unwrap();
            s_p_g.flush().unwrap();
        }
    }

    fn send_flush(&mut self) {
        let mut s_p_g = self.serial.lock().expect("Failed to lock serial in UI thread for button command");
        let mut msg :  Option<MessageType> = Some(MessageType::FlashMemory(ValidFlash::new()));
        let encode_msg = postcard::to_vec::<_, 64>(&msg.unwrap()).unwrap();
        s_p_g.write_all(&encode_msg).unwrap();
        s_p_g.flush().unwrap();
    }

    fn can_transition(&self, from: &str, to: &str, is_calibrated: bool) -> bool {
        match (from, to) {
            ("Safe", "Safe") => true,
            ("Safe", "Manual") => true,
            ("Safe", "Calibrate") => true,
            ("Safe", "Yaw Control") => is_calibrated,
            ("Safe", "Full Control") => is_calibrated,
            ("Safe", "Full Control Raw") => is_calibrated,
            ("Full Control", "Height Control") => true,
            ("Full Control Raw", "Height Control") => true,
            ("Panic", "Safe") => true,
            ("Manual", "Panic") => true,
            ("Yaw Control", "Panic") => true,
            ("Calibrate", "Panic") => true,
            ("Calibrate", "Safe") => true,
            ("Full Control", "Panic") => true,
            ("Full Control Raw", "Panic") => true,
            ("Height Control", "Panic") => true,
            ("Safe", "Wireless") => true,
            ("Wireless", "Safe") => true,
            _ => false
        }
    }

    fn button_change_state(&mut self, state: &str, ui: &mut egui::Ui, is_calibrated: bool) {
        let is_active = self.state.to_string() == state;
        let is_allowed = self.can_transition(&self.state.to_string(), state, is_calibrated);

        let fill = if is_active {
            Color32::from_rgb(144, 248, 144) // Light green if active
        } else if !self.initialized {
            Color32::from_rgb(255, 100, 100) // Red if throttle not yet initialized
        } else if state == "Calibrate" && !is_calibrated {
            Color32::from_rgb(255, 165, 0) // Orange if not yet calibrated
        } else if is_allowed {
            Color32::from_rgb(100, 100, 255) // Blue if transition is allowed
        } else {
            Color32::from_gray(100)
        };

        if state != "Wireless" {
            if ui.add(egui::Button::new(state).fill(fill)).clicked() && is_allowed {
                self.action_state(state);
            }
        }
    }

    fn select_graph(&mut self, ui : &mut egui::Ui){
        ui.horizontal_centered(|ui | {
            if ui.button("Accel/Gyro").clicked(){
                self.imu_graph = true;
            }else if ui.button("Angle").clicked(){
                self.imu_graph = false;
            }
        });
    }


    fn circle_yaw(&mut self, ui: &mut egui::Ui) {
        ui.vertical_centered(|ui|{
            ui.label(egui::RichText::new("----- The rate of change for yaw -----").strong());

            // Reset button for the arrow (for option 2)
            if ui.button("Reset").clicked() {
                let msg = Some(MessageType::ResetYaw(ResetYaw::new()));
                let mut s_p_g = self.serial.lock().expect("Failed to lock serial in UI thread for button command");
                let encode_msg = postcard::to_vec::<_, 64>(&msg.unwrap()).unwrap();
                s_p_g.write_all(&encode_msg).unwrap();
                s_p_g.flush().unwrap();
            }
    
            let (_id, rect) = ui.allocate_space(egui::Vec2::new(110.0, 110.0));
            let painter = ui.painter();
    
            // Make it a white circle
            let center = rect.center();
            let radius = 55.0;
            let color = egui::Color32::WHITE;
            painter.circle_filled(center, radius, color);
    
            // The pointing up arrow reference
            let point_up_arrow = egui::Vec2::new(0.0, -radius);
            painter.arrow(center, point_up_arrow, egui::Stroke::new(2.0, egui::Color32::BLACK));
    
            // The rate of how much it goes in any direction
            let i = self.yaw_angle.pop().unwrap_or([0.0,0.0]);

            let angle_rad = i[1].to_radians();
            let yaw_arrow = egui::Vec2::new(
                (-angle_rad.sin() * radius as f64) as f32,
                (-angle_rad.cos() * radius as f64) as f32
            );
            self.yaw_angle.push(i);
            // Both options need this line below
            painter.arrow(center, yaw_arrow, egui::Stroke::new(2.0, egui::Color32::RED));
        });
    }

    fn motor_image(&mut self, ui: &mut egui::Ui) {
        // Allocate a 160×160 area
        let (_id, rect) = ui.allocate_space(egui::Vec2::new(160.0, 160.0));
        let painter = ui.painter();
    
        let dro_color = Color32::BROWN;       // Brown cross
        let cir_color = Color32::LIGHT_GREEN; // Circle color
        let max_radius: f32 = 20.0;                 
    
        let center = rect.center();
    
        // Draw the cross arms
        let horizontal_rect = egui::Rect::from_center_size(center, egui::Vec2::new(160.0, 10.0));
        let vertical_rect   = egui::Rect::from_center_size(center, egui::Vec2::new(10.0, 160.0));
        painter.rect_filled(horizontal_rect, 0.0, dro_color);
        painter.rect_filled(vertical_rect,   0.0, dro_color);
    
        // Scale motor values to [0, max_radius]
        let largest = *self.motors.iter().max().unwrap_or(&0);
        let motors = self.motors.map(|m| {
            if largest == 0 {
                0.0
            } else {
                (m as f32 * max_radius) / (largest as f32)
            }
        });
    
        // --- Top circle ---
        let radius_top = motors[0].min(max_radius);
        let top_center = egui::pos2(center.x, center.y - 80.0 + radius_top);
        painter.circle_filled(top_center, radius_top, cir_color);
    
        // --- Right circle ---
        let radius_right = motors[1].min(max_radius);
        let right_center = egui::pos2(center.x + 80.0 - radius_right, center.y);
        painter.circle_filled(right_center, radius_right, cir_color);
    
        // --- Bottom circle ---
        let radius_bottom = motors[2].min(max_radius);
        let bottom_center = egui::pos2(center.x, center.y + 80.0 - radius_bottom);
        painter.circle_filled(bottom_center, radius_bottom, cir_color);
    
        // --- Left circle ---
        let radius_left = motors[3].min(max_radius);
        let left_center = egui::pos2(center.x - 80.0 + radius_left, center.y);
        painter.circle_filled(left_center, radius_left, cir_color);
        println!("{:?}", self.motors);
    }

    fn full_control_variables(&mut self, ui: &mut egui::Ui, mut vars_f:((f32,f32,f32),(f32,f32,f32),f32), mut yaw : f32) -> (((f32,f32,f32),(f32,f32,f32),f32),f32){

        ui.vertical(|ui|{
            ui.add_space(20.0);
            ui.horizontal(|ui|{
                let mut s_p_g = self.serial.lock().expect("Failed to lock serial in UI thread for Row");
                ui.label("ROW    [");
                ui.label("P Var =>");


                let entity_r_kp = ui.add(egui::DragValue::new(&mut vars_f.0.0).speed(0.3));

                if entity_r_kp.changed(){
                    let mut msg  = MessageType::ControlPacket(ControlPacket::new(self.yaw_variables, self.yaw_k_variable, self.full_variables));
                    let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
                    s_p_g.write_all(&encode_msg).unwrap();
                    s_p_g.flush().unwrap();
                }

                ui.label("I Var =>");
                let entity_r_ki = ui.add(egui::DragValue::new(&mut vars_f.0.1).speed(0.3));
                if entity_r_ki.changed(){
                    let mut msg  = MessageType::ControlPacket(ControlPacket::new(self.yaw_variables, self.yaw_k_variable, self.full_variables));
                    let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
                    s_p_g.write_all(&encode_msg).unwrap();
                    s_p_g.flush().unwrap();
                }

                ui.label("D Var =>");
                let entity_r_kd = ui.add(egui::DragValue::new(&mut vars_f.0.2).speed(0.3));
                if entity_r_kd.changed(){
                    let mut msg  = MessageType::ControlPacket(ControlPacket::new(self.yaw_variables, self.yaw_k_variable, self.full_variables));
                    let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
                    s_p_g.write_all(&encode_msg).unwrap();
                    s_p_g.flush().unwrap();
                }
                ui.label("   ]");
            });

            ui.horizontal(|ui|{
                ui.label("PITCH   [");
                ui.label("P Var =>");
                let entity_p_kp = ui.add(egui::DragValue::new(&mut vars_f.1.0).speed(0.3));
                let mut s_p_g = self.serial.lock().expect("Failed to lock serial in UI thread for Pitch");
                if entity_p_kp.changed(){
                    let mut msg  = MessageType::ControlPacket(ControlPacket::new(self.yaw_variables, self.yaw_k_variable, self.full_variables));
                    let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
                    s_p_g.write_all(&encode_msg).unwrap();
                    s_p_g.flush().unwrap();
                }

                ui.label("I Var =>");
                let entity_p_ki = ui.add(egui::DragValue::new(&mut vars_f.1.1).speed(0.3));
                if entity_p_ki.changed(){
                    let mut msg  = MessageType::ControlPacket(ControlPacket::new(self.yaw_variables, self.yaw_k_variable, self.full_variables));
                    let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
                    s_p_g.write_all(&encode_msg).unwrap();
                    s_p_g.flush().unwrap();
                }

                ui.label("D Var =>");
                let entity_p_kd = ui.add(egui::DragValue::new(&mut vars_f.1.2).speed(0.3));
                if entity_p_kd.changed(){
                    let mut msg  = MessageType::ControlPacket(ControlPacket::new(self.yaw_variables, self.yaw_k_variable, self.full_variables));
                    let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
                    s_p_g.write_all(&encode_msg).unwrap();
                    s_p_g.flush().unwrap();
                }
                ui.label("   ]");
            });

            ui.horizontal(|ui|{
                ui.label("YAW     [");
                ui.label("P Var =>");
                let entity_y_kp = ui.add(egui::DragValue::new(&mut yaw).speed(0.3));
                let mut s_p_g = self.serial.lock().expect("Failed to lock serial in UI thread for Yaw");
                if entity_y_kp.changed(){
                    let mut msg  = MessageType::ControlPacket(ControlPacket::new(self.yaw_variables, self.yaw_k_variable, self.full_variables));
                    let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
                    s_p_g.write_all(&encode_msg).unwrap();
                    s_p_g.flush().unwrap();
                }
                ui.label("k Var =>");
                let entity_y_kpk : egui::Response;
                entity_y_kpk = ui.add(egui::DragValue::new(&mut self.yaw_k_variable).speed(0.3));
                if entity_y_kpk.changed(){
                    let mut msg  = MessageType::ControlPacket(ControlPacket::new(self.yaw_variables, self.yaw_k_variable, self.full_variables));
                    let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
                    s_p_g.write_all(&encode_msg).unwrap();
                    s_p_g.flush().unwrap();
                }
                ui.label("   ]");
            });
        });
        (vars_f,yaw)
    }

    fn set_control_kalman(&mut self, full : ((f32,f32,f32),(f32,f32,f32),f32), yaw : f32, k_var : (f32,f32)){
        if self.full_variables != full || self.yaw_variables != yaw{
            let mut s_p_g = self.serial.lock().expect("Failed to lock serial in UI thread for retransmitting control variables");
            let mut msg  = MessageType::ControlPacket(ControlPacket::new(self.yaw_variables, self.yaw_k_variable, self.full_variables));
            let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
            s_p_g.write_all(&encode_msg).unwrap();
            s_p_g.flush().unwrap();
        }else if self.kalaman_variables != k_var && (self.kalaman_variables.0.abs() > 0.09375 && self.kalaman_variables.1.abs() > 0.09375) {
            println!("{:?}",self.kalaman_variables);
            let mut s_p_g = self.serial.lock().expect("Failed to lock serial in UI thread for retransmitting control variables");
            let mut msg  = MessageType::KalmanPacket(KalmanPacket::new(self.kalaman_variables));
            let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
            s_p_g.write_all(&encode_msg).unwrap();
            s_p_g.flush().unwrap();
        }
        else{
            println!("THE VALUES MATCH :: {:?} {:?} {:?}",full,yaw,k_var);
        }
    }

    fn kalman_variables(&mut self, ui : &mut egui::Ui){
        ui.vertical(|ui|{
            ui.add_space(25.0);
            let mut s_p_g = self.serial.lock().expect("Failed to lock serial in UI thread for Kalman Values");
            ui.horizontal(|ui|{

                ui.label("Measurment Noise   [");
                let entity_kal_mea = ui.add(egui::DragValue::new(&mut self.kalaman_variables.0).speed(0.3));
                if entity_kal_mea.changed() && (self.kalaman_variables.0.abs() > 0.09375) {
                    let mut msg  = MessageType::KalmanPacket(KalmanPacket::new(self.kalaman_variables));
                    let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
                    s_p_g.write_all(&encode_msg).unwrap();
                    s_p_g.flush().unwrap();
                }
                ui.label("]");
            });

            ui.horizontal(|ui|{

                ui.label("Process Noise             [");
                let entity_kal_proc = ui.add(egui::DragValue::new(&mut self.kalaman_variables.1).speed(0.3));
                if entity_kal_proc.changed() && (self.kalaman_variables.1.abs() > 0.09375) {
                    let mut msg  = MessageType::KalmanPacket(KalmanPacket::new(self.kalaman_variables));
                    let encode_msg = postcard::to_vec::<_, 128> (&msg).unwrap();
                    s_p_g.write_all(&encode_msg).unwrap();
                    s_p_g.flush().unwrap();
                }
                ui.label("]");
            });
        });
    }

    fn motor_speed(&mut self, ui : &mut egui::Ui){

        ui.vertical(|ui|{
                ui.add_space(10.0);

                ui.label(format!("Motor 1 :: {}",self.motors[0]));
                ui.label(format!("Motor 2 :: {}",self.motors[1]));
                ui.label(format!("Motor 3 :: {}",self.motors[2]));
                ui.label(format!("Motor 4 :: {}",self.motors[3]));
        });
    }

    fn trim_values(&mut self, ui : &mut egui::Ui){

        ui.vertical(|ui|{
            ui.add_space(10.0);

            ui.label(format!("Throttle trim (a/z) :: {}",self.throttle_trim));
            ui.label(format!("Roll trim         (</>) :: {}",self.roll_trim));
            ui.label(format!("Pitch trim       (^/v) :: {}",self.pitch_trim));
            ui.label(format!("Yaw trim        (q/w) :: {}",self.yaw_trim));
        });

        ui.add(egui::Separator::default().vertical());

        ui.vertical(|ui|{
            ui.add_space(10.0);

            ui.label(format!("-- Controls PD --"));
            ui.label(format!("Trim yaw P (u/j)"));
            ui.label(format!("Roll/pitch P (i/k)"));
            ui.label(format!("Roll/pitch D (o/l)"));
        });
    }

    fn save_stuff(&mut self, ui : &mut egui::Ui){
        let button = egui::Button::new("Save Values");
        if ui.add(button).clicked(){
            self.write_to_file(MAP_LOCATION);
        }
    }

    fn write_to_file(&mut self, path: &str) -> io::Result<()> {

        self.map_values.insert("Yaw Control", vec![self.yaw_variables]);
        self.map_values.insert("Full Control", vec![self.full_variables.0.0,self.full_variables.0.1,self.full_variables.0.2,self.full_variables.1.0,self.full_variables.1.1,self.full_variables.1.2,self.full_variables.2]);
        self.map_values.insert("Full Control Raw", vec![self.full_variables_raw.0.0,self.full_variables_raw.0.1,self.full_variables_raw.0.2,self.full_variables_raw.1.0,self.full_variables_raw.1.1,self.full_variables_raw.1.2,self.full_variables_raw.2]);
        self.map_values.insert("Kalman Filter", vec![self.kalaman_variables.0,self.kalaman_variables.1]);

        let json = serde_json::to_string_pretty(&self.map_values)
            .map_err(|e| io::Error::new(io::ErrorKind::Other, e))?;

        let mut file = fs::File::create(path)?;
        file.write_all(json.as_bytes())?;
        Ok(())
    }

    /// Read the HashMap from a JSON file
    fn read_from_file(&mut self, path: &str) -> HashMap<String, Vec<f32>> {
        let data = fs::read_to_string(path).unwrap();
        let map : HashMap<String, Vec<f32>> = serde_json::from_str(&data).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e)).unwrap();

        self.yaw_variables = map.get("Yaw Control").unwrap()[0];
        let f_c : &Vec<f32> =  map.get("Full Control").unwrap();
        self.full_variables  = ((f_c[0],f_c[1],f_c[2]),(f_c[3],f_c[4],f_c[5]),f_c[6]);
        let f_c_r : &Vec<f32> =  map.get("Full Control Raw").unwrap();
        self.full_variables_raw  = ((f_c_r[0],f_c_r[1],f_c_r[2]),(f_c_r[3],f_c_r[4],f_c_r[5]),f_c_r[6]);
        let k_v = map.get("Kalman Filter").unwrap();
        self.kalaman_variables = (k_v[0],k_v[1]);
        map
    }

    fn load_or_init(&mut self) -> HashMap<String, Vec<f32>> {
        // Check if the file already exists
        if fs::metadata(MAP_LOCATION).is_ok() {
            // File exists -> read it
            let existing_data = self.read_from_file(MAP_LOCATION);
            return existing_data
        } else {
            // File does not exist -> create a new MyStruct and write it
            return HashMap::from([
                ("Yaw Control".to_string(),       vec![0.0]),
                ("Full Control".to_string(),      vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                ("Full Control Raw".to_string(),  vec![0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                ("Kalman Filter".to_string(),     vec![0.0, 0.0, 0.0]),
            ])
        }
    }

}

impl eframe::App for MyApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {

        // Receive new data from the drone
        if let Ok((
                g_x, g_y, g_z, 
                a_x, a_y, a_z, 
                p_r,time, state, 
                m1, m2, m3, m4,
                full_control,yaw_control, k_vars,
                pressure, battery_voltage, altitude)) = self.control_rx_drone.try_recv() {
            self.set_gyro_values(time, g_x, g_y, g_z);
            self.set_acce_values(time, a_x, a_y, a_z);
            self.set_motor_values(m1, m2, m3, m4);
            self.time = time as f64;
            self.state = state;
            self.set_control_kalman(full_control,yaw_control,k_vars);
            self.pitch_angle.push([time as f64, (p_r.0 * (180.0/3.14159))  as f64]);
            self.roll_angle.push([time as f64, (p_r.1 * (180.0/3.14159)) as f64]);
            self.yaw_angle.push([time as f64,(p_r.2) as f64]);

            self.pressure = pressure;
            self.battery_voltage = battery_voltage;
            self.altitude = altitude;
            self.set_altitude_new(time, altitude);
        }

        // Receive new data from the controller
        if let Ok((pitch, roll, yaw, throttle, initialized)) = self.control_rx_controller.try_recv()  {
            self.set_pitch_rate(pitch);
            self.set_roll_rate(roll);
            self.set_yaw_rate(yaw);
            self.set_throttle_rate(throttle);
            self.set_initialized(initialized);
        }

        // Check for keyboard press
        ctx.input(|i|{
            self.keyboard_commands(i);
        });

        // --- Top panel for heading + buttons ---
        egui::TopBottomPanel::top("top_panel").show(ctx, |ui| {

            ui.vertical_centered(|ui| {
                ui.heading("Group 3 Embedded System Laboratory UI");
                ui.separator();
                ui.horizontal(|ui| {
                    ui.horizontal(|ui| {
                        let font_id = egui::FontId::new(24.0, egui::FontFamily::Proportional);
                        ui.style_mut().text_styles.insert(egui::TextStyle::Button, font_id);

                        self.button_change_state("Safe", ui, self.is_calibrated);
                        self.button_change_state("Manual", ui, self.is_calibrated);
                        self.button_change_state("Panic", ui, self.is_calibrated);
                        self.button_change_state("Calibrate", ui, self.is_calibrated);
                        self.button_change_state("Yaw Control", ui, self.is_calibrated);
                        self.button_change_state("Full Control", ui, self.is_calibrated);
                        self.button_change_state("Full Control Raw", ui, self.is_calibrated);
                        self.button_change_state("Height Control", ui, self.is_calibrated);

                        let can_toggle = matches!(self.state, Safe);

                        let fill_color = if self.wireless_toggled {
                            Color32::from_rgb(100, 100, 255)
                        } else if can_toggle {
                            Color32::from_rgb(255, 165, 0)
                        } else {
                            Color32::from_gray(100)
                        };

                        if ui.add(egui::Button::new("Wireless").fill(fill_color)).clicked() && can_toggle {
                            self.action_state("Wireless")
                        }
                    });


                    ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                        if ui.add(
                            egui::Button::new("Flush flash memory").min_size(egui::Vec2::new(120.0, 35.0))
                        ).clicked() {
                            dbg!("FLUSH MEMORY");
                            self.send_flush();
                        }

                        let battery_voltage: u32 = self.battery_voltage;
                        let pressure: u32 = self.pressure;

                        ui.label(format!("Battery: {} V | Pressure: {} Pa", battery_voltage, pressure));
                    });

                });
            });
        });

        // --- Central panel for the plot and the yaw direction on the bottom ---
        egui::CentralPanel::default().show(ctx, |ui| {
            ui.horizontal_centered(|ui|{
                ui.group(|ui| {
                    self.set_plot_IMU(ui);
                    self.set_plot_angles(ui);

                    ui.vertical_centered_justified(|ui|{
                        self.circle_yaw(ui);

                        if self.state.to_string() != "Safe"{
                            self.motor_image(ui);
                        }
                    });

                });
                ui.add_space(10.0);
            });

        });
        let color_throttle = if self.initialized {
            Color32::GRAY
        } else {
            Color32::RED
        };

        // --- Bottom panel for the sliders ---
        egui::TopBottomPanel::bottom("bottom_panel").show(ctx, |ui| {
            ui.separator();
            ui.horizontal(|ui|{
                ui.vertical(|ui| {
                    ui.add_space(15.0);
                    ui.style_mut().spacing.slider_width = 300.0;
                    ui.style_mut().spacing.slider_rail_height = 20.0;

                    ui.horizontal(|ui| {
                            ui.style_mut().spacing.slider_width = (250.0);
                            ui.add(
                                egui::Slider::new(&mut self.pitch_rate, -100.0..=100.0)
                                    .trailing_fill(true)
                                    .min_decimals(3)
                                    .handle_shape(egui::style::HandleShape::Rect { aspect_ratio: 0.1 })
                            );
                            ui.label(egui::RichText::new("PITCH"));
                    });
                    ui.add_space(5.0);
                    ui.horizontal(|ui| {
                            ui.style_mut().spacing.slider_width = (250.0);
                            ui.add(
                                egui::Slider::new(&mut self.roll_rate, -100.0..=100.0)
                                    .trailing_fill(true)
                                    .min_decimals(3)
                                    .handle_shape(egui::style::HandleShape::Rect { aspect_ratio: 0.1 })
                            );
                            ui.label(egui::RichText::new("ROLL"));
                    });
                    ui.add_space(5.0);

                    ui.horizontal(|ui| {
                        ui.style_mut().spacing.slider_width = (250.0);
                        ui.add(
                            egui::Slider::new(&mut self.yaw_rate, -100.0..=100.0)
                                .trailing_fill(true)
                                .min_decimals(3)
                                .handle_shape(egui::style::HandleShape::Rect { aspect_ratio: 0.1 })
                        );
                        ui.label(egui::RichText::new("YAW"));
                    });
                    ui.add_space(5.0);

                    ui.horizontal( |ui|{
                            ui.style_mut().spacing.slider_width = (250.0);
                            ui.add(
                                egui::Slider::new(&mut self.throttle_rate, 0.0..=100.0)
                                    .min_decimals(3)
                                    .trailing_fill(true)
                                    .handle_shape(egui::style::HandleShape::Rect { aspect_ratio: 0.1 })
                            );
                            ui.label(egui::RichText::new("THROTTLE").color(color_throttle));
                    });
                    ui.add_space(10.0);
                });
                ui.add_space(10.0);
                ui.add(egui::Separator::default().vertical());

                if self.state.to_string() == "Full Control"{
                    (self.full_variables,self.full_variables.2) = self.full_control_variables(ui,self.full_variables,self.full_variables.2);
                } else if self.state.to_string() == "Full Control Raw" {
                    (self.full_variables_raw,self.full_variables_raw.2) = self.full_control_variables(ui,self.full_variables_raw,self.full_variables_raw.2);
                } else {
                    (self.full_variables,self.yaw_variables) = self.full_control_variables(ui,self.full_variables,self.yaw_variables);
                }

                ui.add_space(10.0);
                ui.add(egui::Separator::default().vertical());
                self.kalman_variables(ui);
                ui.add_space(10.0);
                ui.add(egui::Separator::default().vertical());
                self.motor_speed(ui);
                ui.add(egui::Separator::default().vertical());
                self.trim_values(ui);
                ui.add(egui::Separator::default().vertical());
                self.save_stuff(ui);
            });

        });

        ctx.request_repaint();
    }
}

pub fn ui_start(serial : Arc<Mutex<SerialPort>>) -> 
    (
        Sender<((i16,i16,i16,i16,i16,i16,(f32,f32,f32),f32,State,u16,u16,u16,u16,((f32,f32,f32),(f32,f32,f32),f32),f32,(f32,f32),u32,u32,i16))>,
        Sender<(f32, f32, f32, f32, bool)>,
        MyApp
    )
{

    let (tx_drone, rx_drone) :
        (Sender<((i16,i16,i16,i16,i16,i16,(f32,f32,f32),f32,State,u16,u16,u16,u16,((f32,f32,f32),(f32,f32,f32),f32),f32,(f32,f32),u32,u32,i16))>,
         Receiver<((i16,i16,i16,i16,i16,i16,(f32,f32,f32),f32,State,u16,u16,u16,u16,((f32,f32,f32),(f32,f32,f32),f32),f32,(f32,f32),u32,u32,i16))>) = mpsc::channel();

    let (tx_controller, rx_controller) :
        (Sender<(f32, f32, f32, f32, bool)>,
         Receiver<(f32, f32, f32, f32, bool)>) = mpsc::channel();

    let mut my_ui = MyApp::new(rx_drone, rx_controller, serial);
    (tx_drone, tx_controller, my_ui)
}

pub fn ui_run(my_ui: MyApp){
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default().with_inner_size([1550.0, 520.0]),
        ..Default::default()
    };

    eframe::run_native(
        "Embedded System Lab UI",
        options,
        Box::new(|_cc| Ok(Box::new(my_ui))),
    ).expect("Failed to start UI");
}

