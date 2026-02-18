use crate::dep_messages::*;

pub fn process_stream(bit_stream : &[u8]) -> u128 {
    
    let bit_stream = bytes_to_u128(bit_stream);

    for i in 0..128{
        let window = (bit_stream >> i) & 0b1111;
        if window == (MSG_START as u128){
            // let idx = i - HEADER_SIZE;
            // let packet = bit_stream >> ((idx + (128 - idx - 75)));
            // if id as u8 == KEYBOARD_COMMAND_ID{
            //     panic!()
            // }
            //let msg_id = packet >> (75 - 7);
            // let (val, id) = valid_message_id(id);
            // let msg = bit_stream >> (i - KeyboardCommand::get_packet_size());
            // let (a,b,c,d) = KeyboardCommand::getter(msg);
            return bit_stream
        }
    }

    return 0;
}

fn valid_message_id(id : u8) -> (bool, u8){
    match id {
        CHANGE_MODE_ID      => (true, CHANGE_MODE_ID),
        JOYSTICK_COMMAND_ID => (true, JOYSTICK_COMMAND_ID),
        KEYBOARD_COMMAND_ID => (true, KEYBOARD_COMMAND_ID),
        CONTROL_PACKET_ID   => (true, CHANGE_MODE_ID),
        ACK_ID              => (true, ACK_ID),
        _                   => (false, 0b0)
    }
}

fn bytes_to_u128(buf: &[u8]) -> u128 {
    let mut result = 0u128;
    for i in 0..16 {
        let byte = buf.get(i).copied().unwrap_or(0); // Assume padding with zeros
        result = (result << 8) | byte as u128;
    }
    result
}