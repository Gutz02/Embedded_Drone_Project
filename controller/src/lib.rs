use gilrs::{Axis, Event, EventType, Gilrs, GamepadId};

/// Exact ranges are yet to be determined, for
/// ROLL, YAW AND PITCH the ranges are meant as
/// [-RANGE, RANGE] and for RPM its [0, RANGE]
const YAW_RANGE: f32 = 100.0;
const RPM_RANGE: f32 = 100.0;
const ROLL_RANGE: f32 =  100.0;
const PITCH_RANGE: f32 = 100.0;

pub struct GamepadController {
    gilrs: Gilrs,
    active_gamepad: Option<GamepadId>,
    roll: f32,
    pitch: f32,
    yaw: f32,
    rpm: f32,
    panic_b: bool,
    initialized: bool
}

impl GamepadController {
    pub fn new() -> Result<Self, gilrs::Error> {
        let gilrs = Gilrs::new()?;
        Ok(Self {
            gilrs,
            active_gamepad: None,
            roll: 0.0,
            pitch: 0.0,
            yaw: 0.0,
            rpm: 0.0,
            panic_b: false,
            initialized: false,
        })
    }

    pub fn update(&mut self) {

        // Mode 0 -> gamepad emulator controller
        // Mode 1 -> joystick controller
        let mut mode: i32 = 0;

        self.panic_b = false;

        // Check if a controller is connected and if so update which controller is used
        let mut connected = false;
        let mut gamepad_name : String = "".to_string();
        for (_id, gamepad) in self.gilrs.gamepads() {
            connected = true;
            gamepad_name = gamepad.name().to_string();
        }

        if !connected {
            dbg!("No controller connected!");
            self.panic_b = true;
            self.initialized = false;
            gamepad_name = "".to_string();
        } else if gamepad_name == "Logitech Logitech Extreme 3D" {
            mode = 1;
            //dbg!("Real joystick connected! - Mode 1");
        } else {
            mode = 0;
            //dbg!("Emulator joystick connected! - Mode 0");
        }

        while let Some(Event { id, event, .. }) = self.gilrs.next_event() {
            self.active_gamepad = Some(id);
            match event {
                EventType::Disconnected => {
                    dbg!("I got disconnected!");
                    self.panic_b = true;
                    self.initialized = false;
                }
                EventType::ButtonPressed(buttonPressed, code) => {
                    // Emulated on Horipad and PS3, comment out when using real joystick
                    if mode == 0 && code.into_u32() == 65843 {
                        self.panic_b = true;
                    }
                    // Real joystick button
                    if mode == 1 && code.into_u32() == 65824 {
                        self.panic_b = true;
                    }
                }
                EventType::AxisChanged(axisChanged, value, code) => {
                    // Emulated on Horipad and PS3, comment out when using real joystick
                    if mode == 0 && code.into_u32() == 196612 {
                        self.initialized = true;
                        self.rpm = (value.abs())*RPM_RANGE;
                    }
                    // Real joystick + - axis
                    if mode == 1 && code.into_u32() == 196614 {
                        self.initialized = true;
                        self.rpm = (((value * -1.0)*RPM_RANGE) + 100f32) / 2f32;
                    }
                }
                _ => {}
            }
        }

        // Update the x and y of the joystick itself
        if let Some(gamepad) = self.active_gamepad.map(|id| self.gilrs.gamepad(id)) {
            self.roll = (gamepad.value(Axis::LeftStickX)*ROLL_RANGE);
            self.pitch = (gamepad.value(Axis::LeftStickY)*PITCH_RANGE);
            self.yaw = (gamepad.value(Axis::RightZ)*YAW_RANGE);
        }
    }

    // Make it so it only calls everything once
    pub fn get_status(&self) -> Option<(bool,f32,f32,f32,f32, bool)> {
        Some((self.panic_b, self.roll, self.pitch, self.yaw, self.rpm, self.initialized))
    }

    pub fn get_initialized(&self) -> Option<bool> {
        Some(self.initialized)
    }
}