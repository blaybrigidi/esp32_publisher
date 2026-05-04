# DialOG — Wearable Vitals Monitor

DialOG is a small wearable device that continuously measures a patient's heart rate and finger temperature, then sends those readings to a cloud dashboard over WiFi. It is designed with diabetic patients in mind, where early detection of changes in heart rhythm and peripheral temperature can indicate complications before they become serious.

The device is built around an ESP32 microcontroller paired with the MAX30102 optical heart-rate sensor. It collects data for 15 seconds, uploads a summary, sleeps briefly to save battery, then repeats — all without losing its WiFi connection.

---

## Table of Contents

1. [What You Need](#what-you-need)
2. [Project Structure](#project-structure)
3. [Setting Up Your Environment](#setting-up-your-environment)
4. [Configuring Your Credentials](#configuring-your-credentials)
5. [Compiling and Flashing](#compiling-and-flashing)
6. [Monitoring Live Output](#monitoring-live-output)
7. [Understanding the Data](#understanding-the-data)
8. [Adjusting Key Settings](#adjusting-key-settings)
9. [Security Notice](#security-notice)
10. [Troubleshooting](#troubleshooting)
11. [Further Reading](#further-reading)

---

## What You Need

### Hardware

| Component | Purpose |
|---|---|
| ESP32 DOIT DevKit V1 | The main microcontroller that runs everything |
| SparkFun MAX30102 (or compatible) | Reads heart rate from a fingertip using infrared light |
| NTC Thermistor (50 kΩ) | Measures finger surface temperature |
| USB-A to Micro-USB cable | Connects the board to your computer for programming |
| Breadboard + jumper wires | For wiring components together |

### Software

| Tool | Where to get it |
|---|---|
| Visual Studio Code | https://code.visualstudio.com |
| PlatformIO IDE extension | Install from the VS Code extensions panel |
| Git (optional) | https://git-scm.com |

### Cloud Services

| Service | What it does |
|---|---|
| HiveMQ Cloud (free tier) | Receives and stores the readings sent from the device |
| Any MQTT dashboard | Subscribe to the topic to view live data |

---

## Project Structure

```
DialOG/
├── src/
│   ├── main.cpp          ← All device logic lives here
│   ├── secrets.h         ← Your WiFi & MQTT credentials (DO NOT commit this)
│   └── certs.h           ← SSL certificate for secure cloud connection
├── docs/
│   └── MANUAL.md         ← Full user manual
├── platformio.ini        ← Build configuration and library list
└── README.md             ← This file
```

---

## Setting Up Your Environment

1. **Install VS Code** from https://code.visualstudio.com if you haven't already.

2. **Install PlatformIO** by opening VS Code, clicking the Extensions icon on the left sidebar, searching for "PlatformIO IDE", and clicking Install.  Restart VS Code when prompted.

3. **Open the project** — in VS Code go to *File → Open Folder* and select the `DialOG` folder.  PlatformIO will automatically detect the project and download all required libraries (listed below) the first time you build.

### Libraries (installed automatically)

| Library | What it does |
|---|---|
| SparkFun MAX3010x | Talks to the heart-rate sensor |
| ESP32_Thermistor | Reads the temperature sensor |
| PubSubClient | Sends MQTT messages to the cloud |
| ArduinoJson | Formats readings as JSON for the cloud |

---

## Configuring Your Credentials

The device needs your WiFi details and MQTT broker credentials to connect. These live in `src/secrets.h`.

A safe template is provided at `src/secrets.h.example`.  Copy it and fill in your values:

```bash
cp src/secrets.h.example src/secrets.h
```

Then open `src/secrets.h` and fill in each line:

```cpp
#define WIFI_SSID        "YourNetworkName"
#define WIFI_PASSWORD    "YourWiFiPassword"

#define MQTT_BROKER_IP   "xxxx.s1.eu.hivemq.cloud"   // from HiveMQ console
#define MQTT_BROKER_PORT 8883                          // always 8883 for TLS
#define MQTT_USER        "your-mqtt-username"
#define MQTT_PASSWORD    "your-mqtt-password"
#define MQTT_CLIENT_ID   "esp32-sensor-node-01"       // must be unique per device

#define DEVICE_ID        "esp32-sensor-node-01"
#define PATIENT_ID       "your-patient-id"            // used as part of the MQTT topic
```

> `secrets.h` is listed in `.gitignore` and should never be committed to version control.

---

## Compiling and Flashing

1. Plug the ESP32 into your computer via USB.

2. In VS Code, look at the blue status bar at the bottom.  Click the **right-arrow (→) Upload** button, or open the PlatformIO sidebar and click *Upload*.

   Alternatively, use the terminal:
   ```bash
   pio run --target upload
   ```

3. PlatformIO will compile the code, then automatically find the correct USB port and flash the device.  You will see a progress bar in the terminal followed by `SUCCESS`.

4. The device will restart and begin running immediately.

### If the upload fails

- Make sure the USB cable supports data transfer (not charge-only).
- On some ESP32 boards you need to hold the **BOOT** button while the upload starts, then release it.
- Check that no other application (e.g. Arduino IDE) has the serial port open.

---

## Monitoring Live Output

Once flashed, you can watch the device's debug messages in real time.

In VS Code, click the **plug icon (Serial Monitor)** in the blue status bar, or run:

```bash
pio device monitor --baud 115200
```

A healthy startup looks like this:

```
Connecting to YourNetwork........
WiFi connected
IP address: 192.168.1.42
Initializing MAX30102...
Place your index finger on the sensor.
MQTT connected
```

Once your finger is on the sensor:

```
[IR] 87432
[BEAT] delta=812ms raw=73.9
[IR] 91204
IR=91204  | BPM=73.9  | Avg BPM=74  | Temp=33.21°C | SDNN=18.3 | RMSSD=12.1
[MQTT] Publish OK
Entering light sleep for 10 seconds...
Woke from light sleep.
DNS resolved: 18.185.42.10
MQTT connected
```

---

## Understanding the Data

Each reading sent to the cloud contains the following fields:

| Field | What it means |
|---|---|
| `hr` | Heart rate in beats per minute (BPM), averaged over the last 8 valid beats |
| `temp` | Finger surface temperature in °C |
| `hrv_sdnn` | Heart-rate variability — SDNN.  Measures how spread out the beat-to-beat timing is.  Higher is generally healthier. |
| `hrv_rmssd` | Heart-rate variability — RMSSD.  Measures rapid, short-term changes between consecutive beats.  Also higher = healthier. |
| `beat_count` | Number of valid heartbeats captured in the 15-second window |
| `patientId` | Identifies which patient the reading belongs to |
| `deviceId` | Identifies which device sent it |
| `esp_millis` | How many milliseconds the device has been running — useful for spotting gaps |

Readings are sent to the MQTT topic `vitals/<patientId>`, for example `vitals/patient_001`.

---

## Adjusting Key Settings

These two values at the top of `src/main.cpp` are the most likely things you will want to change:

```cpp
// How long (in milliseconds) the device collects data before uploading
const unsigned long PUBLISH_INTERVAL_MS = 15000;  // 15 seconds

// How long the device sleeps between uploads (in microseconds)
#define SLEEP_DURATION_US (10ULL * 1000000ULL)    // 10 seconds (testing)
// For production use 5 minutes:
// #define SLEEP_DURATION_US (300ULL * 1000000ULL)
```

A shorter collection window means more frequent uploads but less data per reading.  A longer sleep saves more battery.

---

## Security Notice

`src/secrets.h` currently contains real credentials in plain text.  If you are sharing this project or pushing it to a public repository, take the following steps:

1. Add `src/secrets.h` to your `.gitignore` file immediately.
2. Rotate your MQTT password in the HiveMQ console.
3. Commit only `src/secrets.h.example` (with placeholder values) so others know the format.

If you are unsure whether credentials have already been exposed, treat them as compromised and generate new ones.

---

## Troubleshooting

**The serial monitor shows "No finger detected" constantly**
Place your finger flat and still on the sensor.  The IR reading must reach at least 50 000 for beat detection to start.  Try pressing slightly harder or wiping the sensor lens with a cloth.

**BPM reads 0 or never updates**
The beat-detection algorithm takes a few seconds to calibrate on each new contact.  Keep your finger still for at least 10 seconds before expecting a reading.

**"WiFi failed — rebooting" appears on startup**
Check that `WIFI_SSID` and `WIFI_PASSWORD` in `secrets.h` are correct and that the network is 2.4 GHz (the ESP32 does not support 5 GHz).

**"MQTT failed, state=-2" in the monitor**
This usually means DNS resolution failed or the broker address in `secrets.h` is wrong.  Double-check `MQTT_BROKER_IP` against your HiveMQ cluster URL.

**Upload fails with "Permission denied" on the port**
On Linux/macOS run `sudo chmod 666 /dev/ttyUSB0` (replace with your actual port).  On macOS you may need to install the CP2102 USB driver.

**SDNN and RMSSD are always 0**
These require at least 2 valid beats to calculate.  If the window is too short or the finger is not detected, they will default to 0.

---

## Further Reading

- Full user manual, sensor explanations, and operational notes: [`docs/MANUAL.md`](docs/MANUAL.md)
- PlatformIO documentation: https://docs.platformio.org
- HiveMQ Cloud getting started: https://www.hivemq.com/mqtt-cloud-broker
- Heart-rate variability explained: https://www.heartmath.org/research/science-of-the-heart/heart-rate-variability
