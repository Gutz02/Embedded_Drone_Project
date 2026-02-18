use core::cell::RefCell;
use tudelft_quadrupel::cortex_m::interrupt;
use tudelft_quadrupel::cortex_m::interrupt::Mutex;
use tudelft_quadrupel::nrf51_pac::{Peripherals, RADIO};
use tudelft_quadrupel::time::{set_tick_frequency, wait_for_next_tick, Instant};
use tudelft_quadrupel::uart::{send_bytes, receive_bytes};
use communication::dep_messages::{MessageType, set_feedback, State, JoystickCommand, ControlPacket, ChangeMode};
use communication::dep_messages::State::Panic;
use crate::util::*;
use communication::initialize::*;
use crate::control::{send, send_match};

#[repr(align(4))]
pub(crate) struct AlignedPacketBuf(pub(crate) [u8; 129]);
static mut RX_BUFFER_PC: AlignedPacketBuf = AlignedPacketBuf([0; 129]);

pub fn control_loop(logging : bool) -> ! {
    set_tick_frequency(250);

    // this is safe since in main the library’s nrf51_peripherals is dropped at the end of initialize,
    // but hardware peripherals are global and still exist in memory.
    let mut nrf51_peripherals = unsafe {Peripherals::steal()};

    init_hfclk(&mut nrf51_peripherals.CLOCK);

    radio_initialize(&mut nrf51_peripherals.RADIO);
    let buffer = unsafe { &mut RX_BUFFER_PC.0 };


    for i in 0.. {
        // let decoded = read_packet();
        //
        // let mut serialize = postcard::to_vec::<_, 128>(&decoded).unwrap();
        //
        // let len = serialize.len();
        // serialize.insert(0, len as u8); // this won't panic
        //
        // init_tx_mode(&mut nrf51_peripherals.RADIO);
        // transmit_packet(&mut nrf51_peripherals.RADIO, &serialize);

        /// Read packet from PC
        let decoded_uart = read_packet();


        // I don't think this panic handle for uart is
        // because even when we disconnect the PC UART the dongle on pc will send Dummy messages which will trigger a panic on the drone

        // match &decoded_uart {
        //     MessageType::Dummy => {
        //         panic_cnt += 1;
        //         if panic_cnt > 50 {
        //             // For panicking when drone disconnects
        //             // pc side
        //             panic_cnt = 0;
        //             decoded_uart = MessageType::ChangeMode(ChangeMode::new(Panic));
        //         }
        //     },
        //     _ => {panic_cnt = 0},
        // }

        send_match(decoded_uart, &mut nrf51_peripherals.RADIO, true);

        /// Receive data from drone side
        buffer.fill(0);

        init_rx_mode(&mut nrf51_peripherals.RADIO);

        let packet = read_packet_wireless(&mut nrf51_peripherals.RADIO, buffer);

        /// SEND BACK TO PC
        if logging{
            send_match(packet, &mut nrf51_peripherals.RADIO, false);
        }

        // wait until the timer interrupt goes off again
        // based on the frequency set above
        wait_for_next_tick();
    }
    panic!();
}