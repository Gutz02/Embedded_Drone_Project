use nrf51_hal::pac::{CLOCK, RADIO};


/// Start the 16 MHz high-frequency clock (HFCLK)
pub fn init_hfclk(clock: &mut CLOCK) {
    // Start HFCLK from external crystal
    clock.tasks_hfclkstart.write(|w| unsafe { w.bits(1) });
    // Wait until started
    while clock.events_hfclkstarted.read().bits() == 0 {}
    // Clear the event
    clock.events_hfclkstarted.write(|w| unsafe {w.bits(0)});
}

/// Configure the RADIO peripheral for proprietary protocol
pub fn radio_initialize(radio: &mut RADIO) {
    // Power the RADIO on if not already
    radio.power.write(|w| w.power().enabled());

    // Clear any leftover events (helps avoid stale event flags)
    radio.events_end.write(|w| unsafe {w.bits(0)});
    radio.events_disabled.write(|w| unsafe {w.bits(0)});
    radio.events_ready.write(|w| unsafe {w.bits(0)});

    // Frequency = 2400 + 0 MHz = 2400 MHz (lowest channel)
    radio.frequency.write(|w| unsafe { w.bits(0) });

    // Radio mode = 1 Mbps Nordic proprietary (nrf_250kbit)
    radio.mode.write(|w| w.mode().nrf_2mbit());

    // Packet configuration:
    // PCNF0: 8-bit length field no S0/S1.
    radio.pcnf0.write(|w| unsafe {
        w.lflen().bits(8)   // 8 bits for the length field
            .s0len().bit(false)
            .s1len().bits(0)
    });

    // PCNF1:
    //   maxlen = 128 bytes
    //   statlen = 0
    //   balen = 4 (so base address is 4 bytes, plus 1-byte prefix)
    //   no data whitening (whiteen = disabled).
    radio.pcnf1.write(|w| unsafe {
        w.maxlen().bits(129)
            .statlen().bits(0)
            .balen().bits(4)
            .endian().little()
            .whiteen().disabled()
    });

    // Set base address (4 bytes).
    // With balen=4, the radio will use these 4 bytes + the 1-byte prefix
    radio.base0.write(|w| unsafe { w.bits(0x00000000) });
    radio.prefix0.write(|w| unsafe {
        w.ap0().bits(0x41)
    });

    // Configure CRC:
    // - 2-byte (16-bit) CRC
    // - Include address field in CRC
    radio.crccnf.write(|w| w.len().two().skipaddr().include());
    // Polynomial = 0x11021 ( x^16 + x^12 + x^5 + 1)
    radio.crcpoly.write(|w| unsafe { w.bits(0x11021) });
    // CRC init value = 0xFFFF (common default)
    radio.crcinit.write(|w| unsafe { w.bits(0xFFFF) });

    radio.pcnf1.modify(|_, w| w.whiteen().enabled());
    radio.datawhiteiv.write(|w| unsafe { w.bits(0x40) }); // example init

    radio.shorts.write(|w|
        w.ready_start().enabled()
            .end_disable().enabled()
    );
}


pub fn init_tx_mode(radio: &mut RADIO) {
    radio.txpower.write(|w| w.txpower()._0d_bm());

    // TX address selection
    radio.txaddress.write(|w| unsafe {w.txaddress().bits(0)});

}

/// Put the RADIO into TX mode and send one packet.
pub fn transmit_packet(radio: &mut RADIO, packet: &[u8]) {
    // Clear old events
    radio.events_end.write(|w| unsafe { w.bits(0) });
    radio.events_disabled.write(|w| unsafe { w.bits(0) });

    // Set the packet pointer
    let ptr = packet.as_ptr() as u32;
    radio.packetptr.write(|w| unsafe { w.bits(ptr) });

    // Enable TX
    radio.tasks_txen.write(|w| unsafe { w.bits(1) });

    // // Wait for END event
    // while radio.events_end.read().bits() == 0 {}
    //
    // // Wait for DISABLED state
    while radio.events_disabled.read().bits() == 0 {}
    }

    pub fn receive_packet(radio: &mut RADIO, packet: &mut [u8]) {
        // Clear old events
        radio.events_end.write(|w| unsafe { w.bits(0) });
        radio.events_disabled.write(|w| unsafe { w.bits(0) });

        // Set the packet pointer
        let ptr = packet.as_ptr() as u32;
        radio.packetptr.write(|w| unsafe { w.bits(ptr) });

        // Enable RX
        radio.tasks_rxen.write(|w| unsafe { w.bits(1) });

        // Wait for END event
        //while radio.events_end.read().bits() == 0 {}

        // Wait for DISABLED state
        while radio.events_disabled.read().bits() == 0 {}
    }

pub fn init_rx_mode(radio: &mut RADIO) {
     // RX address selection
    radio.rxaddresses.write(|w| w.addr0().enabled());
}




