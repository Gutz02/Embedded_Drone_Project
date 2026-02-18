
use tudelft_quadrupel::fixed::{types::extra::U7, FixedI16, FixedI32};
use crate::filtering::{filter_util::*, atan_lut::Lut};
use crate::control::DTIME;

const GYRO_NOISE_SQUARED  : Fix16_7 = Fix16_7::MAX; // Actual Value should be 16.4*16.4=268.96, MAX is 255.99
const ACCEL_NOISE_SQUARED : Fix16_7 = Fix16_7::from_bits(0b0000001000000000);
const DT_SQUARED          : Fix16_7 = Fix16_7::from_bits(1);

pub struct Kalman{
    pub k_angle_roll : Fix16_7,
    pub k_uncertainty_roll : Fix16_7,
    pub k_angle_pitch : Fix16_7,
    pub k_uncertainty_pitch : Fix16_7,
    pub k_offset : AngleRaw,
    pub k_trim   : (Fix16_7, Fix16_7),
    pub lut : Lut,
}

impl Kalman{

    pub fn new(lut : Lut) -> Kalman{
        Kalman{
            k_angle_roll       : Fix16_7::from_num(0.0),
            k_uncertainty_roll : Fix16_7::from_num(0.0),
            k_angle_pitch      : Fix16_7::from_num(0.0),
            k_uncertainty_pitch: Fix16_7::from_num(0.0),
            k_offset           : AngleRaw::new(None, None, None),
            k_trim             : (Fix16_7::ZERO, Fix16_7::ZERO),
            lut 
        }
    }

    pub fn set_offset(&mut self){
        self.k_offset = AngleRaw::new(
            Some(self.k_angle_pitch), 
            Some(self.k_angle_roll), 
            None
        )
    }

    pub fn set_trim(&mut self, roll_t : i16, pitch_t : i16){
        self.k_trim = (
            Fix16_7::from_num(roll_t),
            Fix16_7::from_num(pitch_t)
        )
    }

    pub fn correct_for_offset(&mut self, pitch: Fix16_7, roll : Fix16_7) -> AngleRaw {
        AngleRaw::new(
            Some(pitch - self.k_offset.pitch), 
            Some(roll - self.k_offset.roll),  
            None
        )
    }

    pub fn reset(&mut self){
        self.k_angle_pitch = Fix16_7::from_num(0.0);
        self.k_angle_roll  = Fix16_7::from_num(0.0);
        self.k_uncertainty_pitch = Fix16_7::from_num(0.0);
        self.k_uncertainty_roll = Fix16_7::from_num(0.0);
    }

    pub fn kalman_predict(
        &mut self,
        roll_state : Fix16_7, roll_uncertainty: Fix16_7, roll_input: Fix16_7, roll_measurement: Fix16_7,
        pitch_state: Fix16_7, pitch_uncertainty: Fix16_7, pitch_input: Fix16_7, pitch_measurement: Fix16_7
    ) -> AngleRaw {
        let dt = Fix16_7::from_num(DTIME);
    
        let mut new_roll_state:Fix32_14 = Fix32_14::from_num(roll_state.saturating_add(dt.saturating_mul(roll_input+self.k_trim.0)));
        let mut new_pitch_state: Fix32_14 = Fix32_14::from_num(pitch_state.saturating_add(dt.saturating_mul(pitch_input+self.k_trim.1)));

        
        let roll_uncertainty_pred:Fix16_7 = roll_uncertainty.saturating_add(
            GYRO_NOISE_SQUARED
        );
        let pitch_uncertainty_pred:Fix16_7 = pitch_uncertainty.saturating_add(
            GYRO_NOISE_SQUARED 
        );

        let k_gain_roll : Fix32_14 = Fix32_14::from_num(roll_uncertainty_pred.saturating_div(
            roll_uncertainty_pred.saturating_add(ACCEL_NOISE_SQUARED)
        ));

        let k_gain_pitch : Fix32_14 = Fix32_14::from_num(pitch_uncertainty_pred.saturating_div(
            pitch_uncertainty_pred.saturating_add(ACCEL_NOISE_SQUARED)
        ));

        let error_roll : Fix32_14 = Fix32_14::from_num(roll_measurement)  - new_roll_state;
        let error_pitch: Fix32_14 = Fix32_14::from_num(pitch_measurement) - new_pitch_state;

        new_roll_state = new_roll_state + k_gain_roll * (error_roll);
        new_pitch_state = new_pitch_state + k_gain_pitch * (error_pitch);

        let updated_roll_uncertainty = (Fix32_14::from_num(1) - k_gain_roll).saturating_mul(
            Fix32_14::from_num(roll_uncertainty_pred));
        let updated_pitch_uncertainty: Fix32_14 = FixedI32::from_num(1) - k_gain_pitch.saturating_mul(
            Fix32_14::from_num(pitch_uncertainty_pred));

        
        self.k_angle_roll = Fix16_7::from_num(new_roll_state);
        self.k_uncertainty_roll = Fix16_7::from_num(updated_roll_uncertainty);
        self.k_angle_pitch = Fix16_7::from_num(new_pitch_state);
        self.k_uncertainty_pitch = Fix16_7::from_num(updated_pitch_uncertainty);
        self.correct_for_offset(
            self.k_angle_pitch, 
            self.k_angle_roll
        )
    }
    
    pub fn square_accel (&mut self, acc:& AccelFiltered) -> [Fix32_14 ; 3]{
        let  acc_0_2 : Fix32_14 = acc.x.wide_mul(acc.x);
        let  acc_1_2 : Fix32_14 = acc.y.wide_mul(acc.y);
        let  acc_2_2 : Fix32_14 = acc.z.wide_mul(acc.z);
        [acc_0_2, acc_1_2, acc_2_2]
    }

    pub fn angle_roll(&mut self, acc:&AccelFiltered, acc_squared : &[Fix32_14 ; 3]) -> Fix16_7 {

        let s : Fix32_14 = acc_squared[0].saturating_add(acc_squared[2]).sqrt();
        let acc_1 : Fix32_14 = Fix32_14::from_num(acc.y);

        if s.abs() < EPSILON {  
            return Fix16_7::from_num(0.0)
        }
        self.lut.lut_atan(acc_1.saturating_div(s))
    }

    pub fn angle_pitch(&mut self, acc:&AccelFiltered, acc_squared : &[Fix32_14;3]) -> Fix16_7 {

        let s : Fix32_14 = acc_squared[1].saturating_add(acc_squared[2]).sqrt();
        let acc_0 : Fix32_14 = Fix32_14::from_num(acc.x);

        if s.abs() < EPSILON {  
            return Fix16_7::from_num(0.0)
        }
        self.lut.lut_atan(acc_0.saturating_div(s))
    }

    pub fn angle_yaw(&self, yaw_rate : Fix16_7, yaw_input : Fix16_7, dt : Fix16_7) -> Fix16_7 {
        yaw_input.saturating_add(
            yaw_rate.saturating_mul(dt)
        )
    }

    pub fn packet(&mut self) -> (f32,f32){
        (16.4,2.0)
    }

}


