use tudelft_quadrupel::fixed::{FixedI16, types::extra::U7};
use tudelft_quadrupel::mpu::structs::{Accel, Gyro};
use crate::filtering::filter_util::*;

pub struct ButterStack{
    pub n : u8,
    accel_x_butter : ButterWorth,
    accel_y_butter : ButterWorth,
    accel_z_butter : ButterWorth,
    gyro_x_butter : ButterWorth,
    gyro_y_butter : ButterWorth,
    gyro_z_butter : ButterWorth,
}

impl ButterStack {
    pub fn new(a : [Fix16_7;2], b : [Fix16_7;2])->ButterStack{
        ButterStack{
            n : 0,
            accel_x_butter : ButterWorth::new(a, b),
            accel_y_butter : ButterWorth::new(a, b),
            accel_z_butter : ButterWorth::new(a, b),
            gyro_x_butter  : ButterWorth::new(a, b),
            gyro_y_butter  : ButterWorth::new(a, b),
            gyro_z_butter  : ButterWorth::new(a, b),
        }
    }
    pub fn filter_raw(&mut self, accel : Accel, gyro : Gyro) -> (AccelFiltered, GyroFiltered) { 
        let accel_x : Fix16_7 = self.accel_x_butter.filter(accel.x);
        let accel_y : Fix16_7 = self.accel_y_butter.filter(accel.y);
        let accel_z : Fix16_7 = self.accel_z_butter.filter(accel.z);
        let gyro_x : Fix16_7 = self.gyro_x_butter.filter(gyro.x);
        let gyro_y : Fix16_7 = self.gyro_y_butter.filter(gyro.y);
        let gyro_z : Fix16_7 = self.gyro_z_butter.filter(gyro.z);
        let accel_filtered = AccelFiltered {x : accel_x, y:accel_y, z:accel_z};
        let gyro_filtered  = GyroFiltered {x : gyro_x, y:gyro_y, z:gyro_z};

        (accel_filtered,gyro_filtered)
    }

}

pub struct ButterWorth{
    a      : [Fix16_7;2],
    b      : [Fix16_7;2],
    x      : (Fix16_7,Fix16_7),
    y      : (Fix16_7,Fix16_7),
}

impl ButterWorth{
    pub fn new(a : [Fix16_7;2], b : [Fix16_7;2]) -> ButterWorth{
        ButterWorth{
            a,
            b,
            x       : (Fix16_7::from_num(0),Fix16_7::from_num(0)),
            y       : (Fix16_7::from_num(0),Fix16_7::from_num(0)),
        }
    }

    pub fn filter(&mut self, x_i : i16) -> Fix16_7 { 
        let MAX : i16 = Fix16_7::MAX.to_num();
        let scaled_x_i = x_i/MAX;
        let x_i_fix : Fix16_7 = Fix16_7::from_num(scaled_x_i);

        let y_0_fix : Fix16_7 =
            self.b[0].saturating_mul(x_i_fix) 
            .saturating_add( // +
                self.b[1].saturating_mul(self.x.0)) 
            .saturating_sub( // -
                self.a[1].saturating_mul(self.y.0)); 


        self.x.1 = self.x.0;
        self.x.0 = x_i_fix;
        self.y.1 = self.y.0;
        self.y.0 = y_0_fix;

        y_0_fix
    }

}