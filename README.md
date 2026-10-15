# Solar-Powered IoT Water Level Monitor

ESP32 firmware for a solar-powered drainage water-level monitoring device. The
firmware reads battery, solar, light, current, and ultrasonic water-level
sensors; publishes readings to Firebase Realtime Database; stores readings in
LittleFS when connectivity is unavailable; and adapts its reporting frequency
to battery voltage and predicted solar energy.

The sketch is intended for the final-year project:

> Machine Learning-Based Energy Prediction and Adaptive Power Scheduling for a
> Solar-Powered IoT Drainage Water Level Monitoring Device

## Features

- Firebase Realtime Database publishing over HTTPS.
- Current reading stored at `/current` and historical readings appended to
  `/history`.
- LittleFS JSON Lines (`.jsonl`) offline buffer for failed or unavailable
  uploads.
- On-device linear regression model for predicted incoming solar power.
- NTP time synchronization for the model's hour-of-day input.
- Adaptive operating modes:
  - **NORMAL**: reads every 2 seconds and publishes every 5 seconds.
  - **POWER_SAVE**: reads every 2 seconds and publishes every 15 seconds.
  - **CRITICAL**: Wi-Fi is normally disabled and readings are buffered; the
    device wakes every 5 minutes to attempt a status upload.
- Flood override: water levels at or above `35 cm` force NORMAL mode so alerts
  continue even when the battery is low.
- Relay control that disables charging when the battery reaches `4.20 V`.

## Hardware and pin map

The firmware is written for an ESP32 and uses the following GPIO assignments:

| Component | ESP32 GPIO | Notes |
| --- | ---: | --- |
| Battery voltage | 34 | Analog input through the configured voltage divider |
| Solar voltage | 35 | Analog input through the configured voltage divider |
| Solar current CT sensor | 33 | EmonLib input |
| LDR / light sensor | 32 | Analog input |
| Ultrasonic trigger | 22 | Digital output |
| Ultrasonic echo | 23 | Digital input |
| Charging relay | 21 | Active-low: LOW = relay on, HIGH = relay off |

The ultrasonic sensor is calibrated for a `50 cm` mounting height. Check the
voltage-divider ratios and calibration constants in the sketch before using
different hardware.

## Required software

- Arduino IDE or PlatformIO.
- ESP32 Arduino board support.
- The following libraries:
  - `Wire`
  - `WiFi`
  - `HTTPClient`
  - `WiFiClientSecure`
  - `LittleFS`
  - `ArduinoJson`
  - `EmonLib`

The first five listed Arduino/ESP32 libraries are included with the ESP32
board package. Install `EmonLib` and `ArduinoJson` through the Arduino Library
Manager if they are not already available.

## Configuration

Before uploading, review these values in
[`esp32-firmware-firebase.ino`](./esp32-firmware-firebase.ino):

```cpp
const char* ssid     = "your-wifi-network";
const char* password = "your-wifi-password";
const char* firebaseHost =
    "https://your-project-default-rtdb.firebaseio.com";
```

Do not commit real Wi-Fi credentials or unrestricted Firebase endpoints to a
public repository. For a shared or production deployment, move credentials
into a local, ignored configuration file or use a provisioning mechanism.
`WiFiClientSecure` currently calls `setInsecure()`, which skips TLS certificate
verification and should be replaced with certificate validation for production
use.

The Firebase Realtime Database must allow the ESP32 to write to the configured
paths, or the rules must be paired with an appropriate authentication scheme.

## Uploading

1. Open `esp32-firmware-firebase.ino` in the Arduino IDE.
2. Select an ESP32 board and the correct serial port.
3. Install the required libraries.
4. Set the Wi-Fi and Firebase configuration.
5. Select the appropriate upload speed and upload the sketch.
6. Open Serial Monitor at **115200 baud**.

On startup the device mounts LittleFS, initializes the ADC and sensors,
connects to Wi-Fi, and starts NTP synchronization. If LittleFS cannot be
mounted, the firmware attempts to format it and mount it again.

## Firebase payload

Each reading contains fields similar to:

```json
{
  "battery_voltage": 3.85,
  "battery_soc": 71,
  "solar_voltage": 12.40,
  "solar_current": 0.315,
  "solar_power": 3.9,
  "water_level": 12.5,
  "light_raw": 2450,
  "relay_state": 1,
  "mode": "NORMAL",
  "flood_alert": false,
  "predicted_power": 2.87,
  "hour": 13.50,
  "timestamp": 0
}
```

`timestamp` is populated from the ESP32 system clock after NTP
synchronization. Before time is available, the model uses its fallback hour
value and the timestamp may not represent wall-clock time.

## Operating thresholds

| Setting | Value |
| --- | ---: |
| Sensor read interval | 2 seconds |
| NORMAL publish interval | 5 seconds |
| POWER_SAVE publish interval | 15 seconds |
| CRITICAL wake interval | 5 minutes |
| NORMAL battery threshold | `>= 3.70 V` |
| POWER_SAVE battery range | `3.30 V` to `< 3.70 V` |
| CRITICAL battery threshold | `< 3.30 V` |
| Prediction POWER_SAVE threshold | `< 0.5 W` |
| Flood threshold | `>= 35 cm` |
| Offline buffer limit | approximately 200 KB |

If the flood threshold is also used by a dashboard, keep the dashboard and
firmware values synchronized.

## Troubleshooting

- **No Firebase data:** verify Wi-Fi credentials, database rules, the Firebase
  URL, and the Serial Monitor HTTP response.
- **Readings remain offline:** check LittleFS capacity and wait for a
  successful Wi-Fi connection; buffered lines are uploaded in small batches.
- **Incorrect voltage:** verify the voltage-divider resistor values and the
  corresponding calibration constants.
- **Incorrect water level:** verify the ultrasonic wiring, mounting height,
  trigger/echo pins, and the sensor's measurement range.
- **No current reading:** verify the CT sensor wiring and tune
  `CT_CALIBRATION` for the installed sensor.
- **Unexpected power mode:** inspect battery voltage, predicted power, and
  flood status in the Serial Monitor.

## File

- [`esp32-firmware-firebase.ino`](./esp32-firmware-firebase.ino) — complete
  ESP32 firmware.
