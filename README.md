# Anti-Theft Backpack Guardian (CC3200)

## Lab Report
- See `Anti_Theft_Guardian_Lab_Report.md` for a formal report-style document that can be exported to PDF.

## Overview
Anti-Theft Backpack Guardian is an embedded security system that helps protect backpacks and luggage in public spaces (libraries, cafes, airports, stations).  
It monitors motion while armed, raises a local buzzer alarm, shows status on an OLED screen, and sends an AWS cloud alert with GPS coordinates when theft-like movement is detected.

## Problem Statement
Backpacks and suitcases are often stolen when temporarily unattended.  
This project provides:
- Fast local alert (buzzer + OLED)
- Remote alert through AWS
- Arming/disarming interaction through IR remote and reset button

## Core Features
- Arm/Disarm using IR remote
- Arming grace period before monitoring starts
- Motion/lift detection using onboard accelerometer (I2C)
- Bag-open detection using LDR module digital output (GPIO)
- Alert state with buzzer pattern
- OLED feedback for all states (DISARMED, ARMING, ARMED, ALERT)
- GPS parsing from NMEA RMC over UART1
- HTTPS POST to AWS IoT Thing Shadow (used for downstream email notification flow)
- Reset button to disarm from alert

## System States
- `DISARMED`: system idle, no theft monitoring
- `ARMING`: countdown/grace period after lock command
- `ARMED`: continuous motion detection active
- `ALERT`: buzzer active + cloud notification + GPS shown on OLED

## Hardware Used
- TI CC3200 LaunchPad
- Onboard accelerometer (I2C)
- IR receiver module + AT&T remote
- 128x128 OLED display (SSD1351, SPI)
- Buzzer (GPIO)
- GPS module (UART1, NMEA output)
- Reset/disarm push button

## Protocol/Interface Mapping
- `I2C`: accelerometer readout
- `SPI (GSPI)`: OLED graphics/status
- `UART1`: GPS input
- `GPIO`: IR input, button input, buzzer output
- `Wi-Fi + TLS/HTTPS`: AWS IoT cloud event posting

## Typical Workflow
1. Power device and connect Wi-Fi.
2. Press IR lock command to arm.
3. Wait for arming grace period.
4. If movement/lift is detected while armed:
   - Device enters ALERT
   - Buzzer sounds
   - OLED shows alert + GPS status
   - Cloud event is posted to AWS
5. Disarm via IR unlock or reset button.

## Cloud Alert Payload
The firmware reports an event to AWS IoT Thing Shadow, typically including:
- `event` (e.g., `BACKPACK_ALERT`)
- `lat`, `lon` (when GPS fix is available)

## Project Requirement Alignment
This project is built to align with EEC172 final-project style requirements:
- Sensor-driven embedded behavior
- Multiple hardware interfaces/protocols
- User interactivity (IR + button + display)
- Network/web-service integration (AWS HTTPS)

Reference:  
https://tailailihe.github.io/UCDavis-EEC172-Lab-Manual/labs/final-project.html

## Build/Run Notes
- Toolchain: TI Code Composer Studio (CC3200 SDK)
- Update credentials and endpoint macros in `main.c` before flashing:
  - `WIFI_SSID`, `WIFI_PASS`, `WIFI_SEC`
  - `SERVER_NAME`, `THING_NAME`
- Ensure pin wiring matches `pinmux.c`.

## AWS IoT Certificate Setup (CC3200)
- Convert downloaded AWS IoT PEM files to DER:
  - Root CA (`AmazonRootCA1.pem`) -> `rootCA.der`
  - Device certificate (`*-certificate.pem.crt`) -> `client.der`
  - Private key (`*-private.pem.key`) -> `private.der`
- Upload DER files to CC3200 serial flash as **user files**:
  - `/cert/rootCA.der`
  - `/cert/client.der`
  - `/cert/private.der`
- Firmware expects those exact paths in `main.c` (`AWS_CA_FILE`, `AWS_CLIENT_CERT`, `AWS_PRIVATE_KEY`).
- Local project copies are stored in `cert/`:
  - `cert/rootCA.der`
  - `cert/client.der`
  - `cert/private.der`

## AWS Email Alert Path
- Device publishes alert state to AWS IoT Thing Shadow over HTTPS/TLS.
- Configure AWS IoT Rule to trigger SNS/Email when reported `event` is `BACKPACK_ALERT`.
- Payload includes `event`, `gps_fix`, optional `lat/lon`, and `uptime_ms`.

## Future Enhancements
- Add bag-open detection via light sensor
- Add timestamp in alert payload
- Improve anti-false-trigger motion filtering
- Mobile app dashboard and geofencing support
