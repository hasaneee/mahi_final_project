# Anti Theft Guardian Device - Lab Report

## 1. Abstract
This project implements an embedded anti-theft device for backpacks and luggage using the TI CC3200 platform. The system monitors motion while armed, activates a buzzer alarm on suspicious movement, shows state on an OLED display (optional), and sends cloud alerts through AWS. GPS data from a u-blox NEO-6M module is parsed over UART to include location when a valid fix is available.

## 2. Problem Statement
Personal bags are often left unattended in public spaces such as libraries, cafes, stations, and airports. A low-cost real-time guardian is needed to detect theft-like movement and immediately notify the owner through local and remote alerts.

## 3. Objectives
- Detect movement/lift events while the system is armed.
- Provide immediate local alert (buzzer + optional OLED state view).
- Provide remote notification over Wi-Fi to AWS IoT.
- Include GPS coordinates in alert payload when available.
- Allow user interaction through IR remote and reset/disarm button.

## 4. Hardware and Interfaces
### 4.1 Components
- TI CC3200 LaunchPad
- Built-in accelerometer (I2C)
- u-blox NEO-6M GPS module (UART1, 9600 bps)
- Buzzer (GPIO)
- IR receiver (GPIO, optional)
- SSD1351 OLED display over SPI (optional)
- Reset/disarm push button (GPIO)

### 4.2 Protocol Usage
- I2C: motion sensor readout
- UART1: GPS NMEA stream (`$GPRMC/$GNRMC`)
- SPI (GSPI): OLED updates
- GPIO: buzzer, reset button, IR input
- Wi-Fi + TLS/HTTPS: AWS event posting

## 5. System Architecture
The firmware is organized as a finite state machine:
- `DISARMED`: idle, waiting for arm command.
- `ARMING`: grace delay before surveillance starts.
- `ARMED`: periodic motion sampling active.
- `ALERT`: buzzer pattern active and cloud alert sent.

Supporting modules:
- Sensor polling and threshold filtering for movement.
- GPS UART parser for fix validity and coordinates.
- Cloud HTTP/TLS client for event upload.
- UI/output layer for OLED and serial status logs.

## 6. Operational Flow
1. Boot device and initialize peripherals.
2. Initialize GPS UART and wait for fix in background.
3. Arm device via IR command.
4. During `ARMED`, sample accelerometer and detect suspicious movement.
5. On trigger, enter `ALERT`, sound buzzer, post cloud event, and show GPS status.
6. Disarm through reset button or IR unlock command.

## 7. Firmware Implementation Notes
- GPS parser accepts `RMC` lines and marks fix valid only when status is `A`.
- Added diagnostics to distinguish:
  - no UART data (`gps_uart=NO-RX`)
  - UART data but no satellite lock (`gps_uart=RX`, `gps_fix=NO`)
- GPS timeout set to 180 seconds to better handle NEO-6M cold start behavior.
- System continues operating even when optional peripherals are disabled.

## 8. Testing and Validation
### 8.1 Unit/Functional Checks
- Verified state transitions (`DISARMED -> ARMING -> ARMED -> ALERT`).
- Verified buzzer toggles only in alert state.
- Verified reset disarms system from alert.
- Verified GPS UART parsing path with live NMEA stream.

### 8.2 GPS Test Outcome (Indoor)
- UART data received successfully from NEO-6M (`gps_uart=RX`).
- No valid fix indoors (`RMC=V`, `GGA quality=0`) which is expected.
- Recommendation: perform lock test under open sky for valid fix acquisition.

### 8.3 Cloud Path
- Wi-Fi and AWS posting are present in firmware.
- In current debug configuration Wi-Fi may be disabled by macro for local testing.

## 9. Requirement Mapping (EEC172-style)
- Sensor-driven behavior: accelerometer + optional GPS/light extensions.
- Multiple protocols: I2C, SPI, UART, GPIO, Wi-Fi/TLS.
- User interactivity: IR arm/disarm + reset button + OLED/serial status.
- Web-service integration: AWS HTTPS event reporting.

## 10. Risks and Mitigations
- Indoor GPS lock failure:
  - Mitigation: longer timeout and no-fix diagnostics.
- False motion triggers:
  - Mitigation: hit-count filtering and threshold tuning.
- Wi-Fi connectivity variability:
  - Mitigation: graceful failure logging and local alarm independence.

## 11. Future Improvements
- Add light sensor for bag-open detection.
- Add RTC/NTP timestamp in payload.
- Add geofence rule and repeated alert policy.
- Add battery voltage and low-power handling.
- Validate checksum for stronger NMEA robustness.

## 12. Conclusion
The Anti Theft Guardian Device meets the target objective of combining local theft detection, alarm response, and cloud-based notification in a single embedded system. The platform is modular, testable, and ready for feature expansion, with GPS-aware diagnostics improving reliability during real deployment.

## 13. References
- EEC172 Final Project guideline: https://tailailihe.github.io/UCDavis-EEC172-Lab-Manual/labs/final-project.html
- Project source files: `main.c`, `pinmux.c`, `README.md`
