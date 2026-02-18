use core::time::Duration;
use tudelft_quadrupel::motor::set_motors;
use tudelft_quadrupel::nrf51_pac::RADIO;
use communication::dep_messages::{ChangeMode, MessageType, State};
use tudelft_quadrupel::time::Instant;
use communication::dep_messages::State::Panic;
use crate::util::*;

pub trait CurrentAction {
    fn action(&mut self, is_calibrated: &mut bool, bluetooth: bool, radio: &mut RADIO, buffer: &mut [u8]) -> MessageType;
}

pub struct DroneFSM {
    state: State,
}

impl DroneFSM {
    pub fn init() -> Self {
        DroneFSM {
            state : State::Safe,
        }
    }

    pub fn transition(&mut self, event: State, is_calibrated: &mut bool) {
        use State::*;
        self.state = match (&self.state, event) {
            (Safe, Safe) => Safe,
            (Safe, Manual) => {
                safe_leds_off();
                manual_leds();
                Manual
            },
            (Safe, Calibration) => {
                safe_leds_off();
                calibrate_leds();
                *is_calibrated = true;
                Calibration
            },
            (Safe, YawControl) => {
                safe_leds_off();
                yaw_leds();
                if *is_calibrated {
                    YawControl
                }
                else {
                    Safe
                }
            },
            (Safe, FullControl) => {
                safe_leds_off();
                full_control_leds();
                if *is_calibrated {
                    FullControl
                }
                else {
                    Safe
                }
            },
            (Safe, FullControlRaw) => {
                safe_leds_off();
                full_control_raw_leds();
                if *is_calibrated {
                    FullControlRaw
                }
                else {
                    Safe
                }
            },
            (FullControl, HeightControl) => {
                full_control_leds_off();
                height_control_leds();
                HeightControl
            }
            (FullControlRaw, HeightControl) => {
                full_control_raw_leds_off();
                height_control_leds();
                HeightControl
            }
            (Panic, Safe) => {
                panic_leds_off();
                safe_leds();
                Safe
            },
            (Manual, Panic) => {
                manual_leds_off();
                self.panic_action()
            },
            (YawControl, Panic) => {
                yaw_leds_off();
                self.panic_action()
            },
            (Calibration, Panic) => {
                calibrate_leds_off();
                self.panic_action()
            },
            (Calibration, Safe) => {
                calibrate_leds_off();
                safe_leds();
                Safe
            },
            (FullControl, Panic) => {
                full_control_leds_off();
                self.panic_action()
            }
            (FullControlRaw, Panic) => {
                full_control_raw_leds_off();
                self.panic_action()
            }
            (HeightControl, Panic) => {
                height_control_leds_off();
                self.panic_action()
            }
            // basically, do nothing
            (x, _) => x.clone(),
        };

    }

    pub fn get_state(&self) -> State{
        self.state.clone()
    }

    pub(crate) fn set_panic(&mut self, is_calibrated: &mut bool){
        if let State::Safe = self.state {}
        else {
            self.transition(Panic, is_calibrated);
        }
    }

    fn panic_action(&mut self) -> State{
        self.state = Panic;
        let mut last_blink = Instant::now();
        let interval = Duration::from_millis(50);
        //panic_graceful_motors();
        set_motors([0,0,0,0]);

        let mut i: u8 = 0;
        while i < 20 {
            let now = Instant::now();
            if now.duration_since(last_blink) >= interval {
                panic_leds();
                last_blink = now;
                i = i + 1;
            }
        };
        State::Safe
    }
}

impl CurrentAction for DroneFSM{
    fn action(&mut self, is_calibrated: &mut bool, bluetooth: bool, radio: &mut RADIO, buffer: &mut [u8]) -> MessageType {
        let packet;
        if bluetooth {
            packet = read_packet_wireless(radio, buffer);
        } else {
            packet = read_packet();
        }

        match &packet {
            MessageType::ChangeMode(cmd) => {
                if ChangeMode::check_packet(&cmd){
                    self.transition(cmd.mode.clone(), is_calibrated);
                }
            }
            _ => {}
        }
        packet
    }
}