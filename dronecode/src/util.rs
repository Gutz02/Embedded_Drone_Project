use core::f64::consts::PI;
use communication::dep_messages::MessageType::Dummy;
use communication::dep_messages::{AdjustedYaw, JoystickCommand, MessageType};
use tudelft_quadrupel::mpu::structs::{Accel, Gyro};
use tudelft_quadrupel::nrf51_pac::RADIO;
use communication::initialize::receive_packet;
use tudelft_quadrupel::{led::Led::{Blue, Green, Red, Yellow}, motor::{get_motor_max, get_motors, set_motors}, uart::receive_bytes};
use tudelft_quadrupel::fixed::{FixedI128, FixedI16, FixedI64};
use tudelft_quadrupel::fixed::types::extra::{U32, U20, U5};
use crate::control::{send,FREQUENCY};
use num_traits::identities::Zero;
use core::ops::{Mul, Neg};

type ControlFixed = FixedI64<U32>;
const FIXED_PI : ControlFixed  = ControlFixed::PI;
const FIXED_C  : ControlFixed  = ControlFixed::from_bits(12 * 65535);
const FIXED_ERR : ControlFixed = ControlFixed::from_bits((0.004 * 65535.0) as i64);

const fn sqrt_lookup(n: i16) -> Option<i16> {
    match n {
        0 => Some(0),
        1..=3 => Some(1),
        4..=8 => Some(2),
        9..=15 => Some(3),
        16..=24 => Some(4),
        25..=35 => Some(5),
        36..=48 => Some(6),
        49..=63 => Some(7),
        64..=80 => Some(8),
        81..=99     => Some(9),
        100..=120 => Some(10),
        121..=143 => Some(11),
        144..=168 => Some(12),
        169..=195 => Some(13),
        196..=224 => Some(14),
        225..=255 => Some(15),
        256..=288 => Some(16),
        289..=300 => Some(17),
        _ => None,
    }
}

pub fn read_packet() -> MessageType {
    let data: &mut [u8; 128] = &mut [0u8 ; 128];
    receive_bytes(data);
    postcard::from_bytes(data).unwrap_or_else(|_| {
        Dummy
    })
}

pub fn read_packet_wireless(radio: &mut RADIO, buffer: &mut [u8]) -> MessageType {
    receive_packet(radio, buffer);

    let mut packet_len = buffer[0] as usize;
    if packet_len > 128 || packet_len <= 0 {
        packet_len = 128;
    }
    let packet = &buffer[1..(1 + packet_len)];

    postcard::from_bytes(packet).unwrap_or_else(|_| Dummy)
}

/// Code for buffered reader
/// Is commented out because it slowed down the system too much
/// and changes were made such that it will give compile errors
/// (due to changes such as the Traits in MessageType check_packet() got reverted
// pub fn read_packet_buffered(mut buffer: &mut [u8; 128]) -> MessageType {
//
//     // not all zero, something is in the buffer
//     while !buffer.iter().all(|&b| b == 0){
//
//         // 'Flush' all 0 elements, a valid message starts with a non-zero
//         if let Some(index) = buffer.iter().position(|&b| b != 0) {
//             let rem = buffer.len() - index;
//             buffer.copy_within(index.., 0);      // Move remaining to front
//             buffer[rem..].fill(0);                  // Clear out stale tail
//         }
//
//         let (msg, remaining) = postcard::take_from_bytes::<MessageType>(buffer)
//             .unwrap_or_else(|_| (Dummy, if buffer.len() > 1 { &buffer[1..] } else { &[] }));
//
//
//         let remaining_len = remaining.len();
//
//         if msg.check_packet() && !matches!(msg, Dummy) {
//             let consumed = buffer.len() - remaining_len;
//             buffer.copy_within(consumed.., 0);
//             buffer[remaining_len..].fill(0);
//             return msg;
//         } else {
//             // Shift by 1 byte
//             if buffer.len() > 1 {
//                 buffer.copy_within(1.., 0);
//                 buffer[buffer.len() - 1] = 0;
//             }
//         }
//
//     }
//     // read bytes for next iteration
//     receive_bytes(buffer);
//
//     // nothing correct is read, default return dummy
//     Dummy
// }


pub fn manual_leds() {
    Yellow.on();
}

pub fn manual_leds_off() {
    Yellow.off();
}

pub fn safe_leds() {
    Blue.on();
}

pub fn safe_leds_off() {
    Blue.off();
}

pub fn yaw_leds() {
    Yellow.on();
}

pub fn yaw_leds_off() {
    Yellow.off();
}

pub fn full_control_leds() {
    Red.on();
}

pub fn full_control_leds_off() {
    Red.off();
}

pub fn full_control_raw_leds() {
    Red.on();
    Green.toggle();
}

pub fn full_control_raw_leds_off() {
    Red.off();
    Green.off();
}

pub fn height_control_leds() {
    Red.on();
    Yellow.on();
}
pub fn height_control_leds_off() {
    Red.off();
    Yellow.off();
}

pub fn calibrate_leds() {
    Blue.on();
    Red.on();
}

pub fn calibrate_leds_off() {
    Blue.off();
    Red.off();
}

pub fn panic_leds() {
    Blue.toggle();
    Red.toggle();
    Green.toggle();
}

pub fn panic_leds_off() {
    Blue.off();
    Red.off();
    Green.off();
}

pub fn panic_graceful_motors(){
    let motors = get_motors();
    let mut m_min = get_motor_max();
    for motor in motors.iter() {
        if *motor < m_min {
            m_min = *motor;
        }
    }
    set_motors([m_min, m_min, m_min, m_min]);
    for omega in (0..m_min).rev() {
        set_motors([omega, omega, omega, omega]);
    }
}

pub fn joystick_set_motors(cmd : &JoystickCommand, throttle_trim: i16, yaw_k: i16) -> [u16;4] {

    if cmd.throttle < 170 { // if throttle is less than 170 then the motors are down
        return [0,0,0,0];
    }


    let mut sqrt_yaw = 0;

    let motor1;
    let motor2;
    let motor3;
    let motor4;


    if cmd.yaw_rate > 0 {
        let a = sqrt_lookup(cmd.yaw_rate);
        match a {
            Some(a) => sqrt_yaw = a * yaw_k,
            None => sqrt_yaw = cmd.yaw_rate,
        }

        motor1 = ((cmd.throttle + throttle_trim - cmd.pitch_rate + sqrt_yaw ))
            .clamp(180, get_motor_max()as i16) as u16; // front
        motor2 = ((cmd.throttle + throttle_trim - cmd.roll_rate - cmd.yaw_rate))
            .clamp(180, get_motor_max()as i16) as u16; // right
        motor3 = ((cmd.throttle + throttle_trim + cmd.pitch_rate + sqrt_yaw))
            .clamp(180, get_motor_max() as i16) as u16; // back
        motor4 = ((cmd.throttle + throttle_trim + cmd.roll_rate - cmd.yaw_rate))
            .clamp(180, get_motor_max()as i16) as u16; // left
    }
    else {
        let a = sqrt_lookup(-cmd.yaw_rate);
        match a {
            Some(a) => sqrt_yaw = -(a * yaw_k),
            None => sqrt_yaw = cmd.yaw_rate,
        }
        motor1 = ((cmd.throttle + throttle_trim - cmd.pitch_rate + cmd.yaw_rate ))
            .clamp(180, get_motor_max()as i16) as u16; // front
        motor2 = ((cmd.throttle + throttle_trim - cmd.roll_rate - sqrt_yaw))
            .clamp(180, get_motor_max()as i16) as u16; // right
        motor3 = ((cmd.throttle + throttle_trim + cmd.pitch_rate + cmd.yaw_rate))
            .clamp(180, get_motor_max() as i16) as u16; // back
        motor4 = ((cmd.throttle + throttle_trim + cmd.roll_rate - sqrt_yaw))
            .clamp(180, get_motor_max()as i16) as u16; // left
    }

    [motor1,motor2,motor3,motor4]
}


/// Smoothens by passing through squared filter,
/// lower differences gives lower rates of compensation
/// while bigger differences give much higher rates of compensation
/// should reduce oscillation
/// values need tweaking tho
/// You can see this as the D-part of the filter
fn yaw_smoothening(yaw: f32) -> f32 {
    if yaw < 0.0 {
        yaw * yaw
    } else {
        -yaw * yaw
    }
}

fn yaw_smoothening_raw(yaw: ControlFixed) -> ControlFixed {
    if yaw < 0.0 {
        yaw * yaw
    } else {
        -yaw * yaw
    }
}



pub fn yaw_set_motors (cmd: &JoystickCommand, yaw: f32, last_yaw: f32, kd:f32, yaw_rate: f32, throttle_trim: i16, yaw_k: i16) -> [u16;4]{

    if cmd.throttle < 170 { // if throttle is less than 170 then the motors are down
        return [0,0,0,0];
    }

    let mut motor1 = (cmd.throttle + throttle_trim - cmd.pitch_rate) as u16;
    let mut motor2 = (cmd.throttle + throttle_trim - cmd.roll_rate) as u16;
    let mut motor3 = (cmd.throttle + throttle_trim + cmd.pitch_rate) as u16;
    let mut motor4 = (cmd.throttle + throttle_trim + cmd.roll_rate) as u16;

    (motor1, motor3) = custom_clamp(motor1,motor3);
    (motor2, motor4) = custom_clamp(motor2,motor4);

    let yaw_max = get_motor_max() as i16 -arr_max([motor1, motor2,motor3,motor4]) as i16;
    let yaw_min = arr_min([motor1, motor2,motor3,motor4])as i16 - 180;

    //let yaw_rate = wrapped_angle_difference(yaw, last_yaw)*(FREQUENCY as f32)/(err_cnt as f32);
    let desired_yaw_rate = cmd.yaw_rate;
    let true_yaw_rate = wrapped_angle_difference(yaw, last_yaw)*(FREQUENCY as f32);
    //send(MessageType::AdjustedYaw(AdjustedYaw::new(kp as i16)));

    let err = kd * (desired_yaw_rate as f32 - true_yaw_rate);


    //let adjusted_yaw = yaw_smoothening(desired_yaw_rate as f32 - err);

    let adjusted_yaw = desired_yaw_rate as f32 - err;



    let adj;
    if yaw_min<yaw_max {
        adj = round_i(adjusted_yaw).clamp(-yaw_min, yaw_min);
    }
    else{
        adj = round_i(adjusted_yaw).clamp(-yaw_max, yaw_max);
    }


    if adj > 0 {
        let mut sqrt_adj = 0;
        let a = sqrt_lookup(adj);
        match a {
            Some(a) => sqrt_adj = a * yaw_k,
            None => sqrt_adj = adj,
        }

        motor1 = (motor1 as i16 + sqrt_adj) as u16;
        motor2 = (motor2 as i16 - adj) as u16;
        motor3 = (motor3 as i16 + sqrt_adj) as u16;
        motor4 = (motor4 as i16 - adj) as u16;
    }
    else {
        let a = sqrt_lookup(adj.abs());
        let mut sqrt_adj;
        match a {
            Some(a) => sqrt_adj = -(a * yaw_k),
            None => sqrt_adj = adj,
        }

        motor1 = (motor1 as i16 + adj) as u16;
        motor2 = (motor2 as i16 - sqrt_adj) as u16;
        motor3 = (motor3 as i16 + adj) as u16;
        motor4 = (motor4 as i16 - sqrt_adj) as u16;
    }

    [motor1,motor2,motor3,motor4]

}

fn arr_min(arr: [u16;4]) -> u16 {
    let mut i_min = arr[0];
    for i in 1..4 {
        if arr[i] < i_min {i_min = arr[i];}
    }
    i_min
}
fn arr_max(arr: [u16;4]) -> u16 {
    let mut i_max = arr[0];
    for i in 1..4 {
        if arr[i] > i_max {i_max = arr[i];}
    }
    i_max
}

fn adjust(wanted: i16, current:f32, last:f32, err_cnt: u16, mut cumul_err: f32, kp: f32, kd: f32, ki: f32, prev_err :  &mut f32) -> f32{
    let wanted_convert = (PI/2.0) as f32 * (wanted as f32 / 200.0);
    let c = 12.0;
    let angle_err: f32 = wrapped_angle_difference(wanted_convert,current);
    cumul_err =cumul_err + angle_err * (1.0/(FREQUENCY as f32)); // * err_cnt as f32;
    let v_err = (angle_err - *prev_err) * (FREQUENCY as f32);
    *prev_err = angle_err;
    kd*v_err + kp*angle_err + ki*cumul_err + c * wanted_convert
}

pub fn full_set_motors (cmd: &JoystickCommand, yaw: f32, last_yaw: f32, full_control:((f32, f32, f32), (f32, f32, f32), f32), err_cnt: u16,
                        roll: f32, last_roll: f32, prev_err_roll : &mut f32,
                        pitch: f32, last_pitch: f32, prev_err_pitch: &mut f32, roll_cum_err: f32, pitch_cum_err: f32,
                        throttle_trim: i16, yaw_k: i16) -> [u16;4]{

    let (r, p, y_kp) = full_control;

    let (r_kp, r_ki, r_kd) =  r;
    let (p_kp, p_ki, p_kd) =  p;

    if cmd.throttle < 170 { // if throttle is less than 170 then the motors are down
        return [0,0,0,0];
    }

    let adjusted_roll = adjust(cmd.roll_rate, roll, last_roll, err_cnt, roll_cum_err, r_kp, r_kd, r_ki, prev_err_roll);
    let adjusted_pitch = adjust(-cmd.pitch_rate, pitch, last_pitch, err_cnt, pitch_cum_err, p_kp, p_kd, p_ki, prev_err_pitch);

    let yaw_rate = wrapped_angle_difference(yaw, last_yaw)*(FREQUENCY as f32);
    let desired_yaw_rate = cmd.yaw_rate;

    let err = y_kp * (desired_yaw_rate as f32 - yaw_rate);
    //let adjusted_yaw = yaw_smoothening(desired_yaw_rate as f32 - err);
    let adjusted_yaw = desired_yaw_rate as f32 - yaw_smoothening(err);


    let mut motor1 = (cmd.throttle + throttle_trim + round_i(adjusted_pitch)).max(0) as u16; // front
    let mut motor2 = (cmd.throttle + throttle_trim - round_i(adjusted_roll)).max(0) as u16;
    let mut motor3 = (cmd.throttle + throttle_trim - round_i(adjusted_pitch)).max(0) as u16; // back
    let mut motor4 = (cmd.throttle + throttle_trim + round_i(adjusted_roll)).max(0) as u16;

    (motor1, motor3) = custom_clamp(motor1,motor3);
    (motor2, motor4) = custom_clamp(motor2,motor4);

    let yaw_max = get_motor_max() as i16 -arr_max([motor1, motor2,motor3,motor4]) as i16;
    let yaw_min = arr_min([motor1, motor2,motor3,motor4])as i16 - 180;


    let adj;
    if yaw_min<yaw_max {
        adj = round_i(adjusted_yaw).clamp(-yaw_min, yaw_min);
    }
    else{
        adj = round_i(adjusted_yaw).clamp(-yaw_max, yaw_max);
    }

    if adj > 0 {
        let mut sqrt_adj = 0;
        let a = sqrt_lookup(adj);
        match a {
            Some(a) => sqrt_adj = a * yaw_k,
            None => sqrt_adj = adj,
        }

        motor1 = (motor1 as i16 + adj) as u16;
        motor2 = (motor2 as i16 - sqrt_adj) as u16;
        motor3 = (motor3 as i16 + adj) as u16;
        motor4 = (motor4 as i16 - sqrt_adj) as u16;
    }
    else {
        let a = sqrt_lookup(adj.abs());
        let mut sqrt_adj;
        match a {
            Some(a) => sqrt_adj = -(a * yaw_k),
            None => sqrt_adj = adj,
        }
        motor1 = (motor1 as i16 + sqrt_adj) as u16;
        motor2 = (motor2 as i16 - adj) as u16;
        motor3 = (motor3 as i16 + sqrt_adj) as u16;
        motor4 = (motor4 as i16 - adj) as u16;
    }

    [motor1,motor2,motor3,motor4]

}

#[inline]
pub fn full_set_motors_raw(
        cmd: &JoystickCommand, 
        yaw: f32, last_yaw: f32, full_control:((f32, f32, f32), (f32, f32, f32), f32), err_cnt: u16,
        roll: f32, last_roll: f32, prev_err_roll : &mut f32,
        pitch: f32, last_pitch: f32, prev_err_pitch: &mut f32,
        roll_cum_err: f32, pitch_cum_err: f32,
        throttle_trim: i16, 
        yaw_k: i16
    ) -> [u16;4]{

    let (r, p, y_kp) = vars_control_fixed(full_control);
    let (r_kp, r_ki, r_kd) =  r;
    let (p_kp, p_ki, p_kd) =  p;

    let (yaw ,last_yaw) = (ControlFixed::from_num(yaw), ControlFixed::from_num(last_yaw));
    let (roll ,last_roll) = (ControlFixed::from_num(roll), ControlFixed::from_num(last_roll));
    let (pitch ,last_pitch) = (ControlFixed::from_num(pitch), ControlFixed::from_num(last_pitch));

    let (mut prev_err_roll ,mut prev_err_pitch) = (ControlFixed::from_num(*prev_err_roll), ControlFixed::from_num(*prev_err_pitch));
    let (roll_cum_err ,pitch_cum_err) = (ControlFixed::from_num(roll_cum_err), ControlFixed::from_num(pitch_cum_err));

    if cmd.throttle < 170 { 
        return [0,0,0,0];
    }

    let adjusted_roll  : ControlFixed = adjust_raw(cmd.roll_rate, roll, last_roll, err_cnt, roll_cum_err, r_kp, r_kd, r_ki, &mut prev_err_roll);
    let adjusted_pitch : ControlFixed = adjust_raw(-cmd.pitch_rate, pitch, last_pitch, err_cnt, pitch_cum_err, p_kp, p_kd, p_ki, &mut prev_err_pitch);

    let yaw_rate : ControlFixed = wrapped_angle_difference_raw(yaw, last_yaw) * ControlFixed::from_num((FREQUENCY as f32));
    let desired_yaw_rate = cmd.yaw_rate;

    let err : ControlFixed = y_kp * (ControlFixed::from_num(desired_yaw_rate) - yaw_rate);
    let adjusted_yaw:ControlFixed = ControlFixed::from_num(desired_yaw_rate) - yaw_smoothening_raw(err);


    let mut motor1 = (cmd.throttle + throttle_trim + round_i_raw(adjusted_pitch)).max(0) as u16; // front
    let mut motor2 = (cmd.throttle + throttle_trim - round_i_raw(adjusted_roll)).max(0) as u16;
    let mut motor3 = (cmd.throttle + throttle_trim - round_i_raw(adjusted_pitch)).max(0) as u16; // back
    let mut motor4 = (cmd.throttle + throttle_trim + round_i_raw(adjusted_roll)).max(0) as u16;

    (motor1, motor3) = custom_clamp(motor1,motor3);
    (motor2, motor4) = custom_clamp(motor2,motor4);

    let yaw_max = get_motor_max() as i16 -arr_max([motor1, motor2,motor3,motor4]) as i16;
    let yaw_min = arr_min([motor1, motor2,motor3,motor4])as i16 - 180;


    let adj;
    if yaw_min<yaw_max {
        adj = round_i_raw(adjusted_yaw).clamp(-yaw_min, yaw_min);
    }
    else{
        adj = round_i_raw(adjusted_yaw).clamp(-yaw_max, yaw_max);
    }

    if adj > 0 {
        let mut sqrt_adj = 0;
        let a = sqrt_lookup(adj);
        match a {
            Some(a) => sqrt_adj = a * yaw_k,
            None => sqrt_adj = adj,
        }


        motor1 = (motor1 as i16 + adj) as u16;
        motor2 = (motor2 as i16 - sqrt_adj) as u16;
        motor3 = (motor3 as i16 + adj) as u16;
        motor4 = (motor4 as i16 - sqrt_adj) as u16;

    }
    else {
        let a = sqrt_lookup(adj.abs());
        let mut sqrt_adj;
        match a {
                Some(a) => sqrt_adj = -(a * yaw_k),
                None => sqrt_adj = adj,
        }
        motor1 = (motor1 as i16 + sqrt_adj) as u16;
        motor2 = (motor2 as i16 - adj) as u16;
        motor3 = (motor3 as i16 + sqrt_adj) as u16;
        motor4 = (motor4 as i16 - adj) as u16;
    }

    [motor1,motor2,motor3,motor4]

}

#[inline(always)]
fn adjust_raw(
    wanted: i16, current:ControlFixed, last:ControlFixed, 
    err_cnt: u16, mut cumul_err: ControlFixed, 
    kp: ControlFixed, kd: ControlFixed, ki: ControlFixed, 
    prev_err :  &mut ControlFixed) -> ControlFixed{
    let wanted_convert : ControlFixed = (FIXED_PI/ControlFixed::from_num(2.0)) * (ControlFixed::from_num(wanted)/ControlFixed::from_num(200.0));
    let angle_err: ControlFixed = wrapped_angle_difference_raw(wanted_convert,current);

    cumul_err = cumul_err + angle_err * FIXED_ERR;// * err_cnt as f32;
    let v_err : ControlFixed = (angle_err - *prev_err) * ControlFixed::from_num((FREQUENCY as f32));
    *prev_err = angle_err;
    kd*v_err + kp*angle_err + ki*cumul_err + FIXED_C * wanted_convert
}

#[inline(always)]
pub fn wrapped_angle_difference_raw(curr_yaw_angle: ControlFixed, prev_yaw_angle: ControlFixed) -> ControlFixed {
    let mut diff : ControlFixed = curr_yaw_angle - prev_yaw_angle;
    if diff > FIXED_PI{
        diff -= ControlFixed::from_num(2) * FIXED_PI;
    }
    else if diff < -PI as f32 {
        diff += ControlFixed::from_num(2) * FIXED_PI;
    }
    diff
}

#[inline(always)]
pub fn round_i_raw(num: ControlFixed) -> i16{
    if num < 0.0 {
        (num - ControlFixed::from_num(0.5)).to_num::<i16>()
    }
    else {
        (num + ControlFixed::from_num(0.5)).to_num::<i16>()
    }
}

#[inline(always)]
fn vars_control_fixed(
    full_control: ((f32, f32, f32), (f32, f32, f32), f32)
) -> ((ControlFixed, ControlFixed, ControlFixed), (ControlFixed, ControlFixed, ControlFixed), ControlFixed) {
    let ((a, b, c), (d, e, f), g) = full_control;
    (
        (
            ControlFixed::from_num(a),
            ControlFixed::from_num(b),
            ControlFixed::from_num(c)
        ),
        (
            ControlFixed::from_num(d),
            ControlFixed::from_num(e),
            ControlFixed::from_num(f)
        ),
        ControlFixed::from_num(g)
    )
}

fn vars_fixed(var : (f32,f32,f32)) -> (ControlFixed,ControlFixed,ControlFixed){
    let (a,b,c) = var;
    (
        ControlFixed::from_num(a),
        ControlFixed::from_num(b),
        ControlFixed::from_num(c),
    )
}

pub fn round_i(num: f32) -> i16{
    if num<0.0 {(num - 0.5) as i16}
    else {(num + 0.5) as i16}
}

pub fn custom_clamp(m1: u16, m2: u16) -> (u16, u16) {
    if m1 < 180 {
        (180, (m2-180+m1).clamp(180, get_motor_max()))
    }
    else if m2 < 180 {
        ((m1-180+m2).clamp(180, get_motor_max()), 180)
    }
    else if m1 > get_motor_max() {
        (get_motor_max(), (m2-get_motor_max()+m1).clamp(180, get_motor_max()))
    }
    else if m2 > get_motor_max() {
        ((m1-get_motor_max()+m2).clamp(180, get_motor_max()), get_motor_max())
    }
    else{
        (m1, m2)
    }
}


pub fn wrapped_angle_difference (curr_yaw_angle: f32, prev_yaw_angle: f32) -> f32 {
    let mut diff = curr_yaw_angle - prev_yaw_angle;
    if diff > PI as f32 {
        diff -= 2.0 * PI as f32 ;
    }
    else if diff < -PI as f32 {
        diff += 2.0 * PI as f32;
    }
    diff
}

fn bar_exp(x: FixedI128<U20>) -> FixedI128<U20> {
    /// Estimates x^1/5.255 using poly expansion
    /// Tuned to be most accurate between x = [0.77, 1.05]
    /// Returns 0.6929 + 0.45009x − 0.169x^2 + 0.02601x^3
    let x_pow2 = x*x;
    let x_pow3 = x_pow2*x;

    FixedI128::<U20>::from_num(0.6929) +
        x.saturating_mul(FixedI128::<U20>::from_num(0.45009)) +
        x_pow2.saturating_mul(FixedI128::<U20>::from_num(-0.169)) +
        x_pow3.saturating_mul(FixedI128::<U20>::from_num(0.02601))
}

fn get_altitude_barometer(pressure: u32, ref_pressure: u32) -> FixedI16::<U5> {
    let p0 = FixedI128::<U20>::from_num(ref_pressure);
    let fixed_pressure = FixedI128::<U20>::from_num(pressure);

    // h = 44330 * [ 1 - ( p / p0 ) ^ ( 1 / 5.255) ]
    let p_div_p0_power_2_dot_5 = bar_exp(fixed_pressure.saturating_div(p0));

    // Calculating full h
    let one_min_p_div_p0_power = FixedI128::<U20>::from_num(1.0).saturating_sub(p_div_p0_power_2_dot_5);
    let constant = FixedI128::<U20>::from_num(4433000.0); // 44330 * 100 (centimeters)
    let result = constant.saturating_mul(one_min_p_div_p0_power);

    FixedI16::<U5>::from_num(result)
}

pub fn get_altitude(dt: f64, pressure: u32, ref_pressure: u32, z_axis: i16, mut vertical_velo: FixedI16::<U5>, mut altitude_est: FixedI16::<U5>) -> (FixedI16<U5>, FixedI16<U5>){
    let baro_altitude = get_altitude_barometer(pressure, ref_pressure);

    // Remove gravity as in a forum told, add this line .saturating_sub(FixedI16::<U5>::from_num(9.81));
    let updated_z = FixedI16::<U5>::from_num(z_axis);
    let fixed_dt = FixedI16::<U5>::from_num(dt);
    // v = integrate a * dt
    vertical_velo = vertical_velo.saturating_add(updated_z.saturating_mul(fixed_dt).saturating_div(FixedI16::<U5>::from_num(1000)));
    // h = integrate v * dt
    altitude_est = altitude_est.saturating_add(vertical_velo.saturating_mul(fixed_dt).saturating_div(FixedI16::<U5>::from_num(1000)));

    let a : FixedI16<U5> = FixedI16::<U5>::from_num(0.25);
    let one_min_a : FixedI16<U5> = FixedI16::<U5>::from_num(1).saturating_sub(a);
    altitude_est = a.saturating_mul(altitude_est).saturating_add(one_min_a.saturating_mul(baro_altitude));
    (vertical_velo, altitude_est)
}

pub fn pred_yaw(gyro_rate : i16, last_yaw : f32, dt : f32) -> f32 {
    last_yaw + ( ((gyro_rate as f32)/16.4 ) * dt)
}

pub fn accel_diff(accel_1 : &Accel , accel_2 : &Accel) -> Accel{
    Accel { 
        x: accel_1.x - accel_2.x, 
        y: accel_1.y - accel_2.y, 
        z: accel_1.z
    }
}


pub fn accel_add(accel_1 : &Accel , accel_2 : &Accel) -> Accel{
    Accel { 
        x: accel_1.x + accel_2.x, 
        y: accel_1.y + accel_2.y, 
        z: accel_1.z 
    }
}

pub fn accel_div(accel_1 : &Accel, div : i16) -> Accel{
    Accel { 
        x: accel_1.x/div, 
        y: accel_1.y/div, 
        z: accel_1.z
    }
}


pub fn gyro_diff(gyro_1 : &Gyro , gyro_2 : &Gyro) -> Gyro{
    Gyro { 
        x: gyro_1.x - gyro_2.x, 
        y: gyro_1.y - gyro_2.y, 
        z: gyro_1.z - gyro_2.z
    }
}

pub fn gyro_add(gyro_1 : &Gyro , gyro_2 : &Gyro) -> Gyro{
    Gyro { 
        x: gyro_1.x + gyro_2.x, 
        y: gyro_1.y + gyro_2.y, 
        z: gyro_1.z + gyro_2.z
    }
}

pub fn gyro_div(gyro_1 : &Gyro, div : i16) -> Gyro{
    Gyro { 
        x: gyro_1.x/div, 
        y: gyro_1.y/div, 
        z: gyro_1.z/div
    }
}
