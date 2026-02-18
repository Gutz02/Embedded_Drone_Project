use core::i16;
use core::fmt;
use serde::{Deserialize, Serialize};

pub const MSG_START : u8 = 0b10011;
pub const MSG_END   : u8 = 0b01101;

// Header size in bits
pub const HEADER_SIZE: u8 = 5;

// SIZE OF NUMERIC VALUES 
pub const NUMERIC_SIZE :u8 = 16;

// Command IDs as 3-bit values (0b000 to 0b111)
pub const COMMAND_ID_SIZE : u8 = 3;
pub const CHANGE_MODE_ID: u8      = 0b001; 
pub const JOYSTICK_COMMAND_ID: u8 = 0b010; 
pub const KEYBOARD_COMMAND_ID: u8 = 0b011; 
pub const CONTROL_PACKET_ID: u8   = 0b100; 
pub const ACK_ID: u8              = 0b111; 


trait CheckPacket {
    fn check_packet(cmd: &MessageType);
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum MessageType {
    Dummy,
    ChangeMode(ChangeMode),
    JoystickCommand(JoystickCommand),
    KeyBoardCommand(KeyboardCommand),
    ControlPacket(ControlPacket),
    KalmanPacket(KalmanPacket),
    FeedBack(FeedBack),
    DMPAngles(DMPAngles),
    LoggingDrone(Logging),
    FlashMemory(ValidFlash),
    TrimDrone(TrimCommand),
    ResetYaw(ResetYaw),
    Bluetooth(Bluetooth),
    AdjustedYaw(AdjustedYaw)
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub enum State {
    Safe,
    Panic,
    Manual,
    Calibration,
    YawControl,
    FullControl,
    FullControlRaw,
    HeightControl,
    ResetYaw
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        let state_str = match self {
            State::Safe => "Safe",
            State::Panic => "Panic",
            State::Manual => "Manual",
            State::Calibration => "Calibrate",
            State::YawControl => "Yaw Control",
            State::FullControl => "Full Control",
            State::FullControlRaw => "Full Control Raw",
            State::HeightControl => "Height Control",
            State::ResetYaw => "Reset Yaw"
        };
        write!(f, "{}", state_str)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ChangeMode{
    pub start : u8,
    pub mode  : State,
    pub end   : bool
}

impl ChangeMode {
    pub fn new(mode: State) -> Self {
        ChangeMode{
            start : 5,
            mode,
            end : true
        }
    }

    pub fn getter(self, msg : u32) -> u8{
        ((0x1F & msg) >> 4) as u8
    }

    pub fn check_packet(cmd: &ChangeMode) -> bool{
        cmd.end && cmd.start == 5
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ResetYaw{
    pub start : u8,
    pub end   : bool,
}

impl ResetYaw {

    pub fn new() -> Self{
        ResetYaw {
            start: 8,
            end: true }
    }

    pub fn check_packet(packet : &ResetYaw) -> bool{
        packet.start == 8 && packet.end
    }

}

#[repr(C)]
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct JoystickCommand{
    pub start      : u8, 
    pub throttle   : i16,
    pub pitch_rate : i16,
    pub yaw_rate   : i16,
    pub roll_rate  : i16,
    pub end        : bool
}


impl JoystickCommand {
    pub fn new(throttle: i16, pitch_rate: i16, yaw_rate: i16, roll_rate: i16) -> Self {
        JoystickCommand {
            start : 8,
            throttle,
            pitch_rate,
            yaw_rate,
            roll_rate,
            end : true
        }
    }

    pub fn check_packet(cmd: &JoystickCommand) -> bool{
        cmd.end && (cmd.start == 8)
    }

}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct KeyboardCommand{
    pub start      : u8,
    pub throttle   : i16,
    pub pitch_rate : i16,
    pub yaw_rate   : i16,
    pub roll_rate  : i16,
    pub end        : bool
}
impl KeyboardCommand {
    pub fn new(throttle: i16, pitch_rate: i16, yaw_rate: i16, roll_rate: i16) -> Self {
        KeyboardCommand {
            start : 8,
            throttle,
            pitch_rate,
            yaw_rate,
            roll_rate,
            end : true 
        }
    }

    pub fn check_packet(cmd: &JoystickCommand) -> bool{
        cmd.end && (cmd.start == 8)
    }

}


#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ControlPacket{
    pub start             : u8,
    pub yaw_proportional  : f32,
    pub yaw_k             : i16,
    pub full_variables    : ((f32,f32,f32),(f32,f32,f32),f32),
    pub end               : bool
}

impl ControlPacket {
    pub fn new(y_p: f32, y_k: i16, f_v : ((f32,f32,f32),(f32,f32,f32),f32) ) -> Self {
        ControlPacket {
            start : 12,
            yaw_proportional : y_p,
            yaw_k : y_k,
            full_variables  : f_v,
            end : true
        }
    }

    // DARIO HELP
    pub fn check_packet(cmd: &ControlPacket) -> bool{
        cmd.end && (cmd.start == 12)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct KalmanPacket{
    pub start               : u8,
    pub k_measurment_noise  : f32,
    pub k_process_noise     : f32,
    pub end                 : bool
}

impl KalmanPacket{

    pub fn new(k_vars : (f32,f32)) -> Self{
        KalmanPacket { 
            start : 64,
            k_measurment_noise : k_vars.0,
            k_process_noise    : k_vars.1,
            end   : true
        }
    }

    pub fn check_packet(cmd: &KalmanPacket) -> bool{
        cmd.end && (cmd.start == 64) && cmd.k_measurment_noise.abs() > 0.1 && cmd.k_process_noise.abs() > 0.1
    }
}


#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct FeedBack{
    start  : u8,
    accel_x: i16,
    accel_y: i16,
    accel_z: i16,
    gyro_x: i16,
    gyro_y: i16,
    gyro_z: i16,

    pitch_roll_yaw : (f32,f32,f32),
    
    m_1 : u16,
    m_2 : u16,
    m_3 : u16,
    m_4 : u16,

    temp: i32,
    pressure: u32,
    battery_voltage: u32,
    state: State,
    altitude: i16,

    full_control : ((f32,f32,f32),(f32,f32,f32),f32),
    yaw_control  : f32,
    kalman_vars  : (f32,f32),
    end : bool
}

impl FeedBack {
    pub fn check_packet(cmd : &FeedBack) -> bool {
        cmd.start == 33 && cmd.end
    }
}

pub fn get_feedback(cmd : &FeedBack, count : &f32) -> 
    (i16,i16,i16,i16,i16,i16,(f32,f32,f32),f32,State,u16,u16,u16,u16,((f32,f32,f32),(f32,f32,f32),f32),f32,(f32,f32),u32,u32,i16){
    (
        cmd.gyro_x,
        cmd.gyro_y,
        cmd.gyro_z,
        cmd.accel_x,
        cmd.accel_y,
        cmd.accel_z,
        cmd.pitch_roll_yaw,
        count.clone(),
        cmd.state.clone(),
        cmd.m_1,
        cmd.m_2,
        cmd.m_3,
        cmd.m_4,
        cmd.full_control,
        cmd.yaw_control,
        cmd.kalman_vars,
        cmd.pressure,
        cmd.battery_voltage,
        cmd.altitude
    )
}

pub fn set_feedback(
    accel_x : i16, 
    accel_y : i16, 
    accel_z : i16, 

    gyro_x : i16, 
    gyro_y : i16, 
    gyro_z : i16, 

    pitch_roll_yaw : (f32,f32,f32),

    state : State,

    m_1 : u16,
    m_2 : u16,
    m_3 : u16,
    m_4 : u16,

    full_control : ((f32,f32,f32),(f32,f32,f32),f32),
    yaw_control  : f32,
    kalman_vars  : (f32,f32),

    pressure : u32,
    battery_voltage : u32,
    altitude: i16
) -> FeedBack {
    FeedBack{
        start : 33,
        accel_x,
        accel_y,
        accel_z,
        gyro_x,
        gyro_y,
        gyro_z,
        pitch_roll_yaw,
        m_1,
        m_2,
        m_3,
        m_4,
        temp: 0,
        pressure,
        battery_voltage,
        state,
        full_control,
        yaw_control,
        kalman_vars,
        altitude,
        end : true
    }
}
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct DMPAngles {
    yaw: f32,
    pitch: f32,
    roll: f32,
}

impl DMPAngles {
    pub fn new (yaw: f32, pitch: f32, roll: f32) -> Self {
        Self {
            yaw,
            pitch,
            roll
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Logging{
    start  : u8,
    accel_x: i16,
    accel_y: i16,
    accel_z: i16,
    gyro_x: i16,
    gyro_y: i16,
    gyro_z: i16,

    m_1 : u16,
    m_2 : u16,
    m_3 : u16,
    m_4 : u16,

    temp: i32,
    pressure: u32,
    battery_voltage: u32,

    end : bool
}

impl Logging {
    pub fn check_packet(cmd : &Logging) -> bool {
        cmd.start == 33 && cmd.end
    }
}

pub fn get_logging(cmd : &Logging, count : &f32) -> (i16,i16,i16,i16,i16,i16,f32,u16,u16,u16,u16){
    (
        cmd.gyro_x,
        cmd.gyro_y,
        cmd.gyro_z,
        cmd.accel_x,
        cmd.accel_y,
        cmd.accel_z,
        count.clone(),
        cmd.m_1,
        cmd.m_2,
        cmd.m_3,
        cmd.m_4,
    )
}

pub fn set_logging(
    accel_x : i16,
    accel_y : i16,
    accel_z : i16,

    gyro_x : i16,
    gyro_y : i16,
    gyro_z : i16,

    m_1 : u16,
    m_2 : u16,
    m_3 : u16,
    m_4 : u16,

) -> Logging {
    Logging{
        start : 33,
        accel_x,
        accel_y,
        accel_z,
        gyro_x,
        gyro_y,
        gyro_z,
        m_1,
        m_2,
        m_3,
        m_4,
        temp: 0,
        pressure: 0,
        battery_voltage: 0,
        end : true
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct ValidFlash{
    pub start : u8,
    pub end   : bool
}

impl ValidFlash {
    pub fn new() -> Self {
        ValidFlash {
            start : 2,
            end : true
        }
    }

    pub fn check_packet(cmd: &ValidFlash) -> bool{
        cmd.end && (cmd.start == 2)
    }

}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct TrimCommand{
    pub start           : u8,
    pub throttle_trim   : i16,
    pub roll_trim       : i16,
    pub pitch_trim      : i16,
    pub yaw_trim        : i16,
    pub end             : bool
}

impl TrimCommand {
    pub fn new(throttle_trim: i16, roll_trim: i16, pitch_trim: i16, yaw_trim: i16) -> Self {
        TrimCommand {
            start : 8,
            throttle_trim,
            roll_trim,
            pitch_trim,
            yaw_trim,
            end : true
        }
    }

    pub fn check_packet(cmd: &TrimCommand) -> bool{
        cmd.end && (cmd.start == 8)
    }

}


#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Bluetooth{
    pub is_initialized: bool
}
impl Bluetooth {
    pub fn new(is_initialized: bool) -> Self {
        Self {
            is_initialized
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct AdjustedYaw {
    adj: f32,
}

impl AdjustedYaw {
    pub fn new (adj: f32) -> Self {
        Self {
            adj
        }
    }
}