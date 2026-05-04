# DialOG — User & Technical Manual

This manual covers everything you need to operate, configure, and understand the DialOG vitals monitor beyond the quick-start guide in the README.

---

## Contents

1. [How the Device Works](#how-the-device-works)
2. [Wiring Guide](#wiring-guide)
3. [The Measurement Cycle](#the-measurement-cycle)
4. [Reading the Serial Output](#reading-the-serial-output)
5. [Understanding Heart-Rate Variability](#understanding-heart-rate-variability)
6. [Temperature Simulation Note](#temperature-simulation-note)
7. [Power and Sleep Behaviour](#power-and-sleep-behaviour)
8. [MQTT & Dashboard Integration](#mqtt--dashboard-integration)
9. [Configuration Reference](#configuration-reference)
10. [Known Limitations](#known-limitations)
11. [Maintenance](#maintenance)

---

## How the Device Works

DialOG operates in a continuous collect-upload-sleep cycle:

```
Power on
    │
    ▼
Connect to WiFi ──── fails? ──▶ reboot and try again
    │
    ▼
Connect to MQTT broker
    │
    ▼
┌─────────────────────────────────────┐
│  COLLECT  (default: 15 seconds)     │
│  • Read IR light from sensor        │
│  • Detect each heartbeat            │
│  • Record beat timing for HRV       │
└──────────────────┬──────────────────┘
                   │
                   ▼
┌─────────────────────────────────────┐
│  UPLOAD                             │
│  • Calculate averages & HRV         │
│  • Format as JSON                   │
│  • Send to MQTT broker              │
└──────────────────┬──────────────────┘
                   │
                   ▼
┌─────────────────────────────────────┐
│  SLEEP  (default: 10 seconds)       │
│  • WiFi stays active internally     │
│  • CPU pauses — saves battery       │
│  • Wakes automatically via timer    │
└──────────────────┬──────────────────┘
                   │
                   ▼
              Repeat cycle
```

The key design decision is **light sleep** rather than a full power-off.  Light sleep keeps the WiFi radio alive in a low-power state, so reconnecting after each wake takes a fraction of a second rather than several seconds.  This makes the collect-upload-sleep cycle fast enough to be clinically useful.

---

## Wiring Guide

### MAX30102 Heart-Rate Sensor

| MAX30102 pin | ESP32 pin |
|---|---|
| VIN | 3.3 V |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

The sensor communicates over I²C — the ESP32's default I²C bus uses pins 21 (data) and 22 (clock).  If you use a different board variant, check which pins it uses for I²C.

### Thermistor (NTC 50 kΩ)

The thermistor is wired as a voltage divider:

```
3.3 V
  │
 [50 kΩ fixed resistor]
  │
  ├──── GPIO 32  (analog input, reads voltage here)
  │
 [NTC thermistor, ~50 kΩ at 25 °C]
  │
GND
```

As temperature rises the thermistor's resistance drops, which raises the voltage at GPIO 32.  The library converts that voltage into a temperature reading automatically.

> **Note:** The thermistor is currently returning invalid data (NaN) in testing.  The firmware falls back to a realistic simulated value — see [Temperature Simulation Note](#temperature-simulation-note).

---

## The Measurement Cycle

### Step 1 — Finger detection

The sensor shines infrared light into the fingertip and measures how much bounces back.  A reading above 50 000 (arbitrary sensor units) means a finger is present with good contact.  Below that threshold, the device resets all beat tracking and waits.

### Step 2 — Beat detection

When a finger is present, the device runs the SparkFun `checkForBeat()` algorithm on each new infrared sample.  This algorithm looks for the characteristic rise-and-fall pattern of blood pulsing through the fingertip.

When a beat is confirmed:
- The time since the last beat is recorded (this is the RR interval, measured in milliseconds)
- BPM is calculated from that gap: `BPM = 60 000 / gap_ms`
- Only beats between 45 and 150 BPM are accepted — anything outside that range is treated as noise or a motion artefact and discarded

Up to 8 beats are kept in a rolling circular buffer.  Older beats fall off as new ones arrive.

### Step 3 — Averaging and HRV calculation

At upload time:
- **Average BPM** is the mean of all stored valid beats
- **SDNN** and **RMSSD** are calculated from the stored RR intervals (see [Understanding HRV](#understanding-heart-rate-variability))

### Step 4 — Upload

The readings are packaged as a JSON object and published to the MQTT broker.  The broker retains the last message, so a subscriber that reconnects will immediately receive the most recent reading without waiting for the next cycle.

### Step 5 — Sleep

The device closes the MQTT session cleanly, enters light sleep for the configured duration, then wakes and reconnects before starting the next collection window.

After waking, the sensor's internal sample buffer (which fills up during the sleep and reconnect period) is cleared so the beat-detection algorithm always starts fresh.

---

## Reading the Serial Output

Connect at **115 200 baud**.  The output uses a consistent format so you can spot problems quickly.

### Prefix legend

| Prefix | What it means |
|---|---|
| `[IR]` | Raw infrared light level printed every 500 ms.  Should fluctuate with your pulse. |
| `[BEAT]` | A heartbeat was detected.  Shows the gap in ms and the instantaneous BPM. |
| `[MQTT]` | Network activity — connection attempts, publish results. |

### Example healthy session

```
WiFi connected
IP address: 192.168.1.42
MQTT connected
[IR] 82341
[IR] 88102
[BEAT] delta=821ms raw=73.1
[IR] 91450
[BEAT] delta=809ms raw=74.2
[IR] 89231
[BEAT] delta=834ms raw=71.9
IR=89231  | BPM=73.1  | Avg BPM=73  | Temp=33.54°C | SDNN=16.2 | RMSSD=11.4
[MQTT] Publish OK
---
Entering light sleep for 10 seconds...
Woke from light sleep.
DNS resolved: 18.185.42.10
[MQTT] Connect attempt 1/3
MQTT connected
[IR] 87002
[BEAT] delta=818ms raw=73.3
```

### Example — no finger detected

```
[IR] 1204
[IR] 1198
IR=1198   | BPM=0.0   | Avg BPM=0   | Temp=30.12°C | No finger detected
[MQTT] Publish OK
```

IR values in the low thousands mean no contact.  The upload still happens (with zero BPM) so the dashboard can show that the sensor is active but unoccupied.

---

## Understanding Heart-Rate Variability

Heart-rate variability (HRV) measures how much the time between heartbeats changes from beat to beat.  A perfectly regular heart that beats every 800 ms exactly has zero variability — and that is actually a bad sign.  A healthy heart subtly speeds up and slows down in response to breathing, activity, and nervous system signals.

Low HRV is associated with stress, fatigue, and several chronic conditions including diabetes.

DialOG reports two standard HRV metrics:

### SDNN (Standard Deviation of NN intervals)

SDNN measures the overall spread of your beat-to-beat intervals.  It is calculated by:
1. Finding the average interval across all stored beats
2. Measuring how far each interval deviates from that average
3. Taking the square root of the average squared deviation

A higher SDNN generally indicates a more adaptable autonomic nervous system.

**Rough reference ranges** (short-term, resting):

| SDNN (ms) | Interpretation |
|---|---|
| > 50 | Healthy variability |
| 20–50 | Moderate — worth monitoring |
| < 20 | Low — may warrant clinical attention |

### RMSSD (Root Mean Square of Successive Differences)

RMSSD zooms in on short-term, rapid changes — specifically the differences between *consecutive* pairs of intervals rather than the whole group.  It is more sensitive to quick fluctuations driven by breathing (respiratory sinus arrhythmia).

A higher RMSSD indicates strong parasympathetic (rest-and-digest) activity, which is associated with good recovery and cardiovascular health.

**Rough reference ranges** (short-term, resting):

| RMSSD (ms) | Interpretation |
|---|---|
| > 40 | Good autonomic tone |
| 20–40 | Moderate |
| < 20 | Reduced — consider follow-up |

> These ranges are indicative only.  Clinical interpretation should always involve a qualified healthcare professional and longer-duration recordings (typically 5 minutes or more).

---

## Temperature Simulation Note

The physical thermistor connected to GPIO 32 is currently returning `NaN` (not a number) during testing — likely a wiring or calibration issue.  Rather than send invalid data, the firmware generates a realistic simulated value.

The simulation models two populations:

**Normal peripheral circulation (70% of readings)**
- Base temperature: ~33.5 °C
- Variation: ±1.5 °C
- This reflects a typical healthy fingertip at rest

**Diabetic peripheral pattern (30% of readings)**
- Base temperature: ~29.5 °C
- Variation: ±2.5 °C
- Diabetic patients often have cooler and more variable fingertip temperatures due to reduced blood flow and nerve damage in the extremities

The 30/70 split is arbitrary and exists to make test data varied enough to validate dashboard behaviour.

**To restore real temperature readings**, fix the thermistor wiring and replace the `simulateTemperature()` call in the upload block with:

```cpp
float tempC = thermistor.readTemperatureC();
```

---

## Power and Sleep Behaviour

### Light sleep vs deep sleep

The device uses **light sleep**, not deep sleep.  The difference matters:

| | Light sleep | Deep sleep |
|---|---|---|
| CPU | Paused | Off |
| RAM | Preserved | Cleared |
| WiFi radio | Low-power standby | Off |
| Wake time | ~500 ms | 3–5 s |
| Power saving | Moderate | Maximum |

Light sleep was chosen because:
1. Reconnecting WiFi from scratch after deep sleep adds 3–5 seconds per cycle, reducing the effective upload rate significantly.
2. RAM is preserved, so beat history and all variables survive the sleep without needing to be rebuilt.

### What happens to millis() during sleep?

`millis()` keeps ticking during light sleep.  This means the timestamp of the last heartbeat (`lastBeat`) becomes stale after sleeping — the gap from the last pre-sleep beat to the first post-wake beat would include the entire sleep duration, producing an impossibly low BPM.

The firmware handles this by resetting `lastBeat = 0` after waking, which tells the beat-detection code to wait for two fresh consecutive beats before calculating a rate.

### Watchdog timer

The ESP32 has a hardware watchdog that reboots the device if the main loop stops responding for too long.  The default timeout is 5 seconds, which is shorter than the time the encrypted MQTT handshake can take.  The firmware extends this to 30 seconds to prevent false reboot triggers during connection establishment.

---

## MQTT & Dashboard Integration

### Broker connection

The device connects to HiveMQ Cloud over TLS (encrypted) on port 8883.  The SSL certificate used to verify the broker's identity is the ISRG Root X1 certificate (the same root authority used by Let's Encrypt), embedded in `src/certs.h`.  This certificate is valid until 2035.

### Topic structure

Readings are published to:

```
vitals/<patientId>
```

For example, if `PATIENT_ID` is set to `patient_001`, readings appear on:

```
vitals/patient_001
```

### Message format

Each message is a JSON object:

```json
{
  "patientId":  "patient_001",
  "deviceId":   "esp32-sensor-node-01",
  "hr":         73.5,
  "temp":       33.21,
  "hrv_sdnn":   18.3,
  "hrv_rmssd":  12.1,
  "beat_count": 12,
  "esp_millis": 47823
}
```

### Subscribing from a dashboard

Any MQTT client can subscribe to receive readings.  To subscribe to all patients at once use the wildcard topic `vitals/#`.

**Using MQTT Explorer (desktop app)**
1. Connect to your HiveMQ cluster address on port 8883 with TLS enabled.
2. Subscribe to `vitals/#`.
3. Readings appear in real time as the device publishes.

**Using Node-RED**
1. Add an MQTT-in node pointed at your broker.
2. Set the topic to `vitals/#`.
3. Connect a JSON parse node, then route values to a dashboard gauge.

### Retained messages

Each reading is published with the `retained` flag set to `true`.  This means the broker saves the last message on each topic and delivers it immediately to any new subscriber — so your dashboard always shows the most recent reading, even if it loads after a sleep gap.

---

## Configuration Reference

All tuneable values are near the top of `src/main.cpp`:

| Setting | Default | Description |
|---|---|---|
| `PUBLISH_INTERVAL_MS` | `15000` | Collection window length in milliseconds |
| `SLEEP_DURATION_US` | `10 000 000` | Sleep duration in microseconds (10 seconds) |
| `RATE_SIZE` | `8` | Number of beats kept in the rolling buffer |
| `THERMISTOR_PIN` | `32` | GPIO pin for the temperature sensor |

Settings defined in `src/secrets.h`:

| Setting | Description |
|---|---|
| `WIFI_SSID` | Name of the WiFi network |
| `WIFI_PASSWORD` | WiFi password |
| `MQTT_BROKER_IP` | HiveMQ cluster hostname |
| `MQTT_BROKER_PORT` | Always 8883 for TLS |
| `MQTT_USER` | MQTT username |
| `MQTT_PASSWORD` | MQTT password |
| `MQTT_CLIENT_ID` | Unique name for this device on the broker |
| `DEVICE_ID` | Device identifier sent in each reading |
| `PATIENT_ID` | Patient identifier — also used as the MQTT topic suffix |

---

## Known Limitations

**Short recording window**
15 seconds provides only 12–18 beats for HRV analysis.  Clinical standards (like the Task Force 1996 guidelines) recommend at least 5 minutes of data for short-term HRV.  The current values are useful for trend monitoring but should not be compared against clinical reference ranges directly.

**Single-patient per device**
Each device is configured for one patient.  To monitor multiple patients, flash separate devices with different `PATIENT_ID` values.

**No local storage**
If the WiFi or broker is unavailable at upload time, that reading is lost.  The device does not queue readings locally.

**Temperature is simulated**
The real thermistor is not yet producing valid readings.  All `temp` values in the current firmware are generated, not measured.

**2.4 GHz WiFi only**
The ESP32 does not support 5 GHz networks.

---

## Maintenance

### Updating firmware

1. Open the project in VS Code with PlatformIO.
2. Make your changes to `src/main.cpp`.
3. Click the upload button or run `pio run --target upload`.
4. The device reboots automatically after flashing.

### Rotating credentials

If you need to change the WiFi password or MQTT password:
1. Update the relevant `#define` in `src/secrets.h`.
2. Reflash the device.

### Replacing the SSL certificate

The `ISRG_ROOT_X1` certificate in `src/certs.h` expires in 2035.  If HiveMQ switches to a different certificate authority before then, or if you migrate to a different broker, replace the certificate block with the new root CA in PEM format.
