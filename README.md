# Digital Embodiment

**Digital Embodiment** is a localized wireless prototype demonstrating remote connection. It bridges the emotional gap caused by long-distance relationships by simulating the physical presence of a person away from home through real-time peer-to-peer motion capture and haptic feedback.

The system consists of two key components:

- **Sensing Band**: Worn by the remote user, it captures motion data along the X, Y, and Z axes using an MPU6050 accelerometer. Data is transmitted via ESP-NOW between two ESP32 modules.

- **Homing Beacon**: Located with the loved one, it receives motion packets and drives three rotating elements (or servos) to recreate gestures in real time. When no motion is detected, the beacon returns to a neutral “flatline” state.

By translating motion into synchronized visual and kinetic feedback, Digital Embodiment offers a tangible way to maintain presence and emotional connection across distances.

---

## Table of Contents

- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Wiring & Pinouts](#wiring--pinouts)
- [Software Dependencies](#software-dependencies)
- [Installation & Deployment](#installation--deployment)
  - [Sensing Band Firmware](#sensing-band-firmware)
  - [Homing Beacon Firmware](#homing-beacon-firmware)
- [Configuration](#configuration)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [License](#license)
- [Author & Acknowledgements](#author--acknowledgements)

---

## Features

- **Real‑time motion capture** via `MPU6050` accelerometer (X, Y, Z axes).
- **Wireless peer‑to‑peer** data transfer using `ESP-NOW` (low latency, no router needed).
- **Three‑axis haptic feedback**: maps motion to three servos or rotating cubes.
- **Movement threshold gating** to ignore noise.
- **Automatic homing**: servos return to neutral when remote user stops moving.
- **Modular design**: separate firmware for sensing band and homing beacon.

## Hardware Requirements

- 2 × ESP32 development boards (e.g. WROOM‑32).
- 1 × MPU6050 accelerometer module.
- 3 × hobby servos or rotating elements for the beacon.
- Jumper wires for I²C (SDA, SCL) and power.
- 5–6 V regulated power supply for servos (external recommended).

## Wiring & Pinouts

### Sensing Band (ESP32 + MPU6050)

- **SDA** → D6 (MPU6050 SDA)
- **SCL** → D7 (MPU6050 SCL)
- **3.3V** → 3.3V (MPU6050 VCC)
- **GND** → GND (MPU6050 GND)

### Homing Beacon (ESP32 + 3 Servos)

- **X‑axis servo** → D4 (Power: 5–6 V, GND)
- **Y‑axis servo** → D5 (Power: 5–6 V, GND)
- **Z‑axis servo** → D6 (Power: 5–6 V, GND)

## Software Dependencies

- Arduino IDE 1.8+ (or PlatformIO).
- ESP32 board package via Boards Manager.
- **Libraries**:
  - `Adafruit_MPU6050`
  - `Adafruit_Sensor`
  - `ESP32Servo`
  - `esp_now` (built-in)
  - `WiFi` (built-in)
  - `Wire` (built-in)

## Installation & Deployment

### Sensing Band Firmware

1. Clone the repo and open the `/sensing-band/` folder.
2. Open **`sensing_band.ino`** in the Arduino IDE.
3. Install required libraries via the Library Manager.
4. Select **ESP32 Dev Module** and correct COM port.
5. Upload the sketch.

### Homing Beacon Firmware

1. Open the `/homing-beacon/` folder.
2. Open **`homing_beacon.ino`**.
3. Set `receiverMac[]` to match the sensing band’s MAC address.
4. Upload to the second ESP32.

## Configuration

- **Movement threshold**: adjust `MOVEMENT_THRESHOLD` to filter noise.
- **Timeout**: modify `TIMEOUT_MS` (ms) to set homing delay.
- **Mapping range**: change `map()` or `mapFloat()` parameters for servo angles.

## Usage

1. Power on both ESP32 devices.
2. Wear the sensing band and move naturally.
3. Observe the homing beacon’s servos or cubes mirror your motions.
4. When motion stops, the beacon returns to neutral after the timeout.

## Troubleshooting

- **No data received**: verify MAC addresses and ESP-NOW initialization.
- **Servo jitter**: confirm a stable external power supply and common ground.
- **Sensor noise**: tweak accelerometer filter bandwidth or increase threshold.

## License

Released under the MIT License. See [LICENSE](LICENSE) for details.

## Author & Acknowledgements

**Jawad Farooq Naik** is a multidisciplinary artist, designer, entrepreneur, and aspiring educator. He holds an MFA in Design from the Dynamic Media Institute at Massachusetts College of Art and Design, and a BFA in Visual Communication Design from the National College of Arts, Lahore, Pakistan.

Inspired by research in embodied interaction, haptic communication, and tangible computing.
