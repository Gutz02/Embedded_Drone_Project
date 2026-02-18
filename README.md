# Embedded Drone Control System (Rust) — Embedded Systems Lab

Real-time control + tooling stack for a tethered quadrotor platform, built during TU Delft’s Embedded Systems Lab.  
The project focuses on: (1) safe state-based control, (2) real-time telemetry + tuning, and (3) robust PC↔drone communication.

## Highlights
- **3-part architecture**: on-drone control (`dronecode`), PC runtime (`runner`), and a real-time **GUI** (`user_interface`)
- **Control loop @ 250 Hz**, plus a **RAW-sensor mode @ 500 Hz**
- Multiple flight/control **modes**: Safe, Panic, Manual, Yaw/Full control, Height control, and a Wireless demo mode
- **Live logging** over UART to CSV, plus optional **flash logging** on the drone
- **Message protocol** with framing + validation to survive noisy serial links
- **Fixed-point support** for low-latency math in RAW mode + filter work

## Repo structure (high-level)
- `dronecode/`  
  Runs on the drone controller board. Contains the main control loop, FSM, filtering, utilities, and attitude state.
- `runner/`  
  Runs on the PC. Handles threads, joystick input processing, logging to CSV, and bridging data to the UI.
- `user_interface/`  
  Real-time UI that plots telemetry and allows live tuning (PID + filter params).
- `communication/`  
  Shared packet/message types + encode/decode logic.
- `controller/`  
  Gamepad handling (Gilrs-based) and joystick state.

## What I worked on (Dario)
- **GUI setup (EGUI)**: real-time plots (IMU + altitude, DMP/RAW angles), yaw visualization, motor-ratio view, and tuning sliders  
  + added UI support for saving control/filter settings to flash.
- **PC↔drone protocol** (with Eric): MessageType-based packets, framing markers, validation IDs, and a buffered UART reader.
- **Fixed-point + filtering support**: fixed-point implementation and work around RAW mode filtering.

## System overview
### Control loop & timing
- Default control runs at **250 Hz**
- **RAW mode** runs at **500 Hz** for higher-rate sensor feedback

### GUI (real-time tuning + visibility)
The GUI supports:
- Live plots for IMU + altitude estimation
- Live plot for attitude angles (DMP or RAW mode)
- Yaw “dial” style visualization (relative to a reference)
- Motor output ratio visualization (helps debug control behavior without flying)
- Sliders for PID gains + filter parameters
- Buttons for flash-save of control/filter settings
- 
<img width="1045" height="391" alt="image" src="https://github.com/user-attachments/assets/52c86810-578e-4914-a272-42edd19816fe" />

<img width="865" height="310" alt="image" src="https://github.com/user-attachments/assets/e72e8ca3-6129-46d2-a5a8-d8749b20e051" />

### Communication protocol (UART)
- Messages are represented as a `MessageType` enum, each variant backed by its own struct
- Receiver-side integrity checks:
  - start/end markers
  - unique IDs for validation
- Joystick commands are polled every **50 ms**
  - dummy messages are sent in-between to keep the link active
- Buffered reader behavior:
  - only reads UART when internal buffer is empty
  - parses/flushes and returns the next valid message
  - invalid messages are dropped; if nothing valid remains, buffer is flushed and `DUMMY` is returned

### Logging
Two logging paths:
1. **Live UART telemetry** → stored on the PC as CSV (created by `runner`)
2. **On-drone flash logging**
   - control loop writes data into flash
   - a `FlashMemory` command triggers a flush of stored entries
   - if flash is full, the flash is formatted (simple approach; circular buffering was considered but not required)

## Modes (user-facing)
- **Safe**: motors off; only allows leaving safe after controller initialization + throttle at zero
- **Panic**: emergency stop behavior (also triggered automatically on disconnect conditions)
- **Manual**: direct motor control from joystick (with basic shaping)
- **Yaw / Full control**: stabilized attitude control (DMP-based)
- **Full control (RAW)**: higher-rate RAW sensor pipeline (500 Hz) + fixed-point math
- **Height control**: uses altitude estimation + control around a target height (joystick sets desired height)
- **Wireless mode (demo)**: nRF51822-based half-duplex link (not flight-ready; too slow)

## Wireless demo (nRF51822)
Wireless uses two flight controller boards as “dongles”:
- Half-duplex: cannot TX/RX at the same time
- PC dongle initiates TX → drone receives → drone TX reply → PC RX  
**Note:** this mode is **not suitable for flight** (too slow); it exists to demonstrate capability.

## Notes / limitations
- Wireless mode is for demonstration only (latency too high for real control)
- RAW mode requires careful tuning; fixed-point parameters were selected based on practical testing
