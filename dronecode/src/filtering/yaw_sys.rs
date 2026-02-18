use communication::dep_messages::JoystickCommand;

const FULL_SCALE_FACTOR : f32 = 16.4;

pub struct YawSys{
    yaw_angle        : f32,
    current_raw_rate : f32,
    last_raw_rate    : f32, 
    dt               : f32,
    counter          : u8,
    trim             : f32,
    cmd              : JoystickCommand
}

impl YawSys{

    pub fn new(time : f32) -> Self{
        YawSys { 
            yaw_angle: 0.0, 
            current_raw_rate : 0.0,
            last_raw_rate : 0.0, 
            dt: time, 
            counter: 0,
            trim : 0.0,
            cmd: JoystickCommand::new(0, 0, 0, 0) 
        }
    }

    pub fn set_trim(&mut self, trim_y : i16){
        self.trim = trim_y as f32
    }

    pub fn decision(&mut self, yaw_rate : i16) -> f32{

        let yaw_rate_f32 = (yaw_rate as f32)/FULL_SCALE_FACTOR; // we factor this down to deg/s (look at datasheet)
        self.last_raw_rate = self.current_raw_rate;
        self.current_raw_rate = yaw_rate_f32 - self.trim* 0.25;
        self.yaw_angle += self.current_raw_rate * self.dt;
        self.counter += 1;
        self.yaw_angle  
    }
 
    pub fn reset(&mut self){
        self.yaw_angle = 0.0;
        self.last_raw_rate = 0.0;
        self.counter = 0;
        self.cmd = JoystickCommand::new(0, 0, 0, 0);
    }

}

