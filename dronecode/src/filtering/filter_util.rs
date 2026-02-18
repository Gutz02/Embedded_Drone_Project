use tudelft_quadrupel::fixed::{FixedI16, FixedI32,types::extra::{U7,U14,U4}};

pub type Fix16_7  = FixedI16<U7>;
pub type Fix32_14 = FixedI32<U14>; 

pub const EPSILON : Fix32_14 = Fix32_14::from_bits(000000000002);

#[derive(Debug, Copy, Clone)]
pub struct AngleRaw{
    pub pitch : Fix16_7,
    pub roll   : Fix16_7,
    pub yaw   : Fix16_7,
}

impl AngleRaw {
    
    pub fn new(pitch : Option<Fix16_7>, roll : Option<Fix16_7>, yaw : Option<Fix16_7>) -> AngleRaw {
        AngleRaw {
            pitch: Fix16_7::from_num(pitch.unwrap_or(FixedI16::from_num(0))), 
            roll: Fix16_7::from_num(roll.unwrap_or(FixedI16::from_num(0))),
            yaw: Fix16_7::from_num(yaw.unwrap_or(FixedI16::from_num(0)))
        }
    }

    pub fn to_f32(&self,yaw:f32)->(f32,f32,f32){
        (
            self.pitch.to_num::<f32>(),
            self.roll.to_num::<f32>(),
            yaw
        )
    }

}

#[derive(Debug, Copy, Clone)]
pub struct AccelFiltered{
    pub x : Fix16_7,
    pub y : Fix16_7,
    pub z : Fix16_7
}

impl AccelFiltered {

    pub fn new() -> AccelFiltered{
        AccelFiltered { 
            x: Fix16_7::from_num(0), 
            y: Fix16_7::from_num(0), 
            z: Fix16_7::from_num(0)
        }
    }
    
}

#[derive(Debug, Copy, Clone)]
pub struct GyroFiltered{
    pub x : Fix16_7,
    pub y : Fix16_7,
    pub z : Fix16_7
}

impl GyroFiltered {

    pub fn new() -> GyroFiltered{
        GyroFiltered { 
            x: Fix16_7::from_num(0), 
            y: Fix16_7::from_num(0), 
            z: Fix16_7::from_num(0)
        }
    }
    
}
