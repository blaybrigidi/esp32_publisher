// ─────────────────────────────────────────────────────────────────────────────
// DialOG — Wearable Vitals Monitor
//
// This device reads a patient's heart rate and finger temperature for 60
// seconds, sends a summary over WiFi to a cloud server, then goes into a
// 5-minute light sleep to save battery while keeping WiFi alive.
// ─────────────────────────────────────────────────────────────────────────────

#include <Wire.h>
#include "MAX30105.h"       // Driver for the heart-rate / blood-oxygen sensor
#include "heartRate.h"      // Beat-detection algorithm for the sensor above
#include <ESP32_Thermistor.h>  // Reads finger temperature from the thermistor on GPIO 32
#include <WiFi.h>
#include <WiFiClientSecure.h>  // Lets us open an encrypted (HTTPS-style) connection
#include <PubSubClient.h>      // MQTT library — sends small messages to a broker
#include <ArduinoJson.h>       // Turns our readings into a JSON string for sending
#include "esp_task_wdt.h"      // Watchdog timer — reboots the device if it freezes
#include "secrets.h"           // WiFi password, MQTT credentials (not in source control)
#include "certs.h"             // SSL certificate so the device trusts the server

// Pin number on the ESP32 where the thermistor is wired
#define THERMISTOR_PIN 32

// ── Timing ────────────────────────────────────────────────────────────────────

// How long (in milliseconds) the device collects readings before sending them.
// 60 seconds gives roughly 50-90 beats at rest, which is what RMSSD needs to
// agree closely with a standard 5-minute recording (a 15 s window gave only
// 12-18 beats and readings that varied by ~50% from one cycle to the next).
// Duty cycle: ~60 s collecting + ~4 s waking/reconnecting out of every ~364 s,
// i.e. the device is awake about 17-18% of the time.
const unsigned long PUBLISH_INTERVAL_MS = 60000;
unsigned long lastPublishTime = 0;

// How long the device sleeps between uploads (in microseconds): 5 minutes.
// For bench testing you can temporarily shorten this, e.g. (10ULL * 1000000ULL),
// but don't commit a short value — the reported duty cycle assumes 5 minutes.
#define SLEEP_DURATION_US (300ULL * 1000000ULL)

// ── Hardware objects ───────────────────────────────────────────────────────────

MAX30105 particleSensor;   // The optical heart-rate sensor (MAX30102 chip)
ESP32_Thermistor thermistor; // Temperature sensor on the finger

// ── Network objects ────────────────────────────────────────────────────────────

WiFiClientSecure espClient;        // Encrypted network connection
PubSubClient mqttClient(espClient); // MQTT messenger that uses that connection

// ─────────────────────────────────────────────────────────────────────────────
// Connect to WiFi.
// Waits up to 20 seconds for a connection.  If the network doesn't respond
// within 10 seconds, it tries reconnecting once before giving up.
// A failed connection reboots the device so it can try again cleanly.
// ─────────────────────────────────────────────────────────────────────────────
void setup_wifi()
{
    const unsigned long TIMEOUT_MS   = 20000; // Give up after 20 s total
    const unsigned long RECONNECT_MS = 10000; // Retry WiFi after 10 s
    const unsigned long DOT_MS       = 500;   // Print a dot every 0.5 s while waiting

    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long start   = millis();
    unsigned long lastDot = start;
    bool nudged = false; // tracks whether we've tried a reconnect yet

    while (WiFi.status() != WL_CONNECTED)
    {
        unsigned long elapsed = millis() - start;

        if (elapsed >= TIMEOUT_MS)
            break; // Took too long — fall through to the error handler

        // Retry once at the halfway point
        if (!nudged && elapsed >= RECONNECT_MS)
        {
            WiFi.reconnect();
            nudged = true;
        }

        // Print a dot so we can see progress on the serial console
        if (millis() - lastDot >= DOT_MS)
        {
            Serial.print(".");
            lastDot = millis();
        }
    }

    Serial.println();

    // If we still aren't connected, print the reason then reboot
    if (WiFi.status() != WL_CONNECTED)
    {
        int s = WiFi.status();
        Serial.print("WiFi failed, status=");
        Serial.print(s);
        Serial.print(" (");
        switch (s)
        {
        case WL_NO_SSID_AVAIL:  Serial.print("SSID not found");    break;
        case WL_CONNECT_FAILED: Serial.print("wrong password");     break;
        case WL_DISCONNECTED:   Serial.print("disconnected/auth");  break;
        case WL_IDLE_STATUS:    Serial.print("idle — timed out");   break;
        default:                Serial.print("unknown");            break;
        }
        Serial.println(") — rebooting...");

        // Brief pause so the message can be read on the console
        unsigned long wait = millis();
        while (millis() - wait < 3000) {}
        ESP.restart();
    }

    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

// ─────────────────────────────────────────────────────────────────────────────
// Package a set of vitals readings into a JSON message and send it to the
// MQTT broker.  The broker keeps the last message so the dashboard always
// shows the most recent reading even if it reconnects after a drop.
// ─────────────────────────────────────────────────────────────────────────────
bool publishReading(float hr, float temp, bool hrvValid, float hrv_sdnn, float hrv_rmssd, byte beatCount)
{
    // Build the MQTT topic from the patient ID, e.g. "vitals/patient_001"
    char topic[64];
    snprintf(topic, sizeof(topic), "vitals/%s", PATIENT_ID);

    // Create the JSON document that will be sent
#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    StaticJsonDocument<256> doc;
#endif
    doc["patientId"]   = PATIENT_ID;
    doc["deviceId"]    = DEVICE_ID;
    doc["hr"]          = hr;          // Heart rate in beats per minute
    doc["temp"]        = temp;        // Finger temperature in °C
    // HRV is only sent when this window passed the quality checks.  A missing
    // value means "not measured" — we never send 0 or a stale number instead.
    if (hrvValid)
    {
        doc["hrv_sdnn"]  = hrv_sdnn;  // Heart-rate variability measure 1
        doc["hrv_rmssd"] = hrv_rmssd; // Heart-rate variability measure 2
    }
    doc["beat_count"]  = beatCount;   // Number of valid beats detected this window
    doc["esp_millis"]  = millis();    // Device uptime — useful for debugging timing

    char buf[256];
    serializeJson(doc, buf);

    Serial.printf("[MQTT] Topic: %s  Payload: %s\n", topic, buf);

    // retained=true means the broker remembers this message and delivers it to
    // any new subscriber immediately, so the dashboard never shows stale data.
    return mqttClient.publish(topic, buf, true);
}

// ─────────────────────────────────────────────────────────────────────────────
// Re-establish the MQTT connection after it drops.
// The brief 1 ms pause lets the processor finish any background housekeeping
// before we start the (slow) encrypted handshake.
// ─────────────────────────────────────────────────────────────────────────────
boolean reconnect()
{
    delay(1); // Let background tasks breathe before the blocking handshake

    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD))
        Serial.println("MQTT connected");
    else
    {
        Serial.print("MQTT failed, state=");
        Serial.println(mqttClient.state());
    }
    return mqttClient.connected();
}


// ─────────────────────────────────────────────────────────────────────────────
// Heart-rate state
//
// Heart rate is reported as the average BPM of the last 8 valid beats.
// ─────────────────────────────────────────────────────────────────────────────
const byte RATE_SIZE = 8;
byte  rates[RATE_SIZE];       // Rolling BPM readings
byte  rateSpot  = 0;          // Write position in the circular buffer
byte  rateCount = 0;          // How many valid slots are filled so far
double lastBeatSample = -1;   // Sample position of the most recent beat; <0 = none yet
float beatsPerMinute = 0;     // BPM from the most recent single beat gap
int   beatAvg   = 0;          // Average BPM across the last RATE_SIZE beats
byte  newBeatsThisCycle = 0;  // Valid beats collected since the last upload

// ─────────────────────────────────────────────────────────────────────────────
// Heart-rate variability (HRV) state
//
// HRV is computed from every beat-to-beat (RR) interval in the current 60 s
// window, in the order the beats happened.  The window starts empty after each
// upload, so a reading never mixes in intervals from a previous cycle.
//
// rrFollowsPrev[i] is true when interval i directly follows interval i-1, i.e.
// no beat was rejected in between.  RMSSD only uses such pairs, so a rejected
// or missed beat never produces a "successive difference" across a gap.
//
// HRV (variability in those gaps) is an indicator of autonomic nervous system
// health.  Diabetic patients often show reduced HRV.
// ─────────────────────────────────────────────────────────────────────────────
const int MAX_RR = 180;           // 60 s at 150 BPM is 150 intervals, plus headroom
float windowRR[MAX_RR];           // RR intervals (ms) accepted in this window
bool  rrFollowsPrev[MAX_RR];      // true if interval i directly follows interval i-1
int   rrCount = 0;                // How many intervals are stored this window
bool  prevBeatAccepted = false;   // Was the previous beat's interval accepted?

// ─────────────────────────────────────────────────────────────────────────────
// Beat timing
//
// Beats are timed on the sensor's own sample clock, not with millis() when the
// loop happens to notice them.  The sensor delivers 100 samples/s (setup()
// defaults: 400 Hz with 4x on-chip averaging), so millis() at detection is only
// good to ~10 ms plus loop jitter — enough to inflate RMSSD noticeably for
// patients whose true RMSSD is low (10-20 ms).
//
// checkForBeat() reports a beat when its filtered signal crosses zero going up,
// between the previous sample and the current one.  Interpolating where that
// crossing falls gives the beat time to a fraction of a sample.  The detection
// itself, and the sensor configuration, are unchanged.
// ─────────────────────────────────────────────────────────────────────────────
extern int16_t IR_AC_Signal_Current;  // filtered values inside checkForBeat()
extern int16_t IR_AC_Signal_Previous; // (globals in the SparkFun heartRate.cpp)

const float NOMINAL_SAMPLE_MS = 10.0f;   // 100 samples per second
const unsigned long FIFO_GAP_MS = 250;   // longer than this and the 32-sample FIFO may have overflowed
uint32_t sampleIndex  = 0;               // samples received since boot
uint32_t anchorSample = 0;               // start of the current unbroken stretch of samples
unsigned long anchorMs = 0;
unsigned long lastDrainMs = 0;
long lastIR = 0;                         // most recent IR value (for diagnostics)
uint16_t gapsThisWindow = 0;             // sample gaps seen this window (for diagnostics)

// Milliseconds per sample.  The sensor's own oscillator may be off by a few
// percent, so this is calibrated against millis() over each unbroken stretch of
// at least 30 s (when the stretch ends: before sleeping, or at a gap) and held
// constant while a window is being timed (a value that changed mid-window would
// distort SDNN).
float sampleMs = NOMINAL_SAMPLE_MS;

// endMs = time of the last sample drain that belongs to the stretch
void calibrateSampleClock(unsigned long endMs)
{
    uint32_t n = sampleIndex - anchorSample;
    if (n < 3000) return; // too short to calibrate reliably
    float measured = (float)(endMs - anchorMs) / (float)n;
    if (measured > 9.0f && measured < 11.0f) // ignore anything implausible
        sampleMs = measured;
}

// Samples were lost (loop stalled, sensor FIFO overflowed, or we just woke up):
// don't measure an interval across the gap, and start a new calibration stretch.
void markSampleGap()
{
    lastBeatSample   = -1;
    prevBeatAccepted = false;
    anchorSample     = sampleIndex;
    anchorMs         = millis();
}

// Quality rules (same as the validation study):
const float ARTEFACT_TOLERANCE = 0.20f; // reject intervals >20% away from the window median
const float MIN_ACCEPTED_SHARE = 0.80f; // at least 80% of intervals must survive that rule
const float MIN_COVERAGE       = 0.70f; // clean intervals must cover >=70% of the window

struct HrvResult
{
    bool  valid; // false = this window didn't pass the quality checks
    float sdnn;  // ms
    float rmssd; // ms
    int   nn;    // number of clean (normal-to-normal) intervals used
};

// Discard everything collected for the current HRV window.
void resetHrvWindow()
{
    rrCount = 0;
    prevBeatAccepted = false;
}

// Median of the stored intervals (insertion sort on a copy; at most MAX_RR values).
float medianRR()
{
    static float sorted[MAX_RR];
    for (int i = 0; i < rrCount; i++)
    {
        float v = windowRR[i];
        int j = i - 1;
        while (j >= 0 && sorted[j] > v)
        {
            sorted[j + 1] = sorted[j];
            j--;
        }
        sorted[j + 1] = v;
    }
    return (rrCount % 2) ? sorted[rrCount / 2]
                         : (sorted[rrCount / 2 - 1] + sorted[rrCount / 2]) / 2.0f;
}

// ─────────────────────────────────────────────────────────────────────────────
// Compute SDNN and RMSSD for the current window.
//
// SDNN  — standard deviation of the clean intervals (sample SD, n-1).
//         Higher values generally mean a healthier, more adaptable rhythm.
// RMSSD — root mean square of differences between *successive* clean
//         intervals.  More sensitive to rapid, breathing-driven changes.
//
// Steps:
//   1. Mark intervals more than 20% away from the window median as artefacts
//      (missed or extra beats).
//   2. Quality gate: if more than 20% were rejected, or the clean intervals
//      cover less than 70% of the window, report no HRV for this window.
//   3. Compute SDNN over the clean intervals, and RMSSD over pairs that are
//      both clean and truly consecutive.
// ─────────────────────────────────────────────────────────────────────────────
HrvResult computeWindowHRV(unsigned long windowMs)
{
    HrvResult r = {false, 0.0f, 0.0f, 0};
    if (rrCount < 3) return r;

    float median = medianRR();
    static bool clean[MAX_RR];
    int   nn  = 0;
    float sum = 0;
    for (int i = 0; i < rrCount; i++)
    {
        clean[i] = fabsf(windowRR[i] - median) < ARTEFACT_TOLERANCE * median;
        if (clean[i])
        {
            nn++;
            sum += windowRR[i];
        }
    }
    r.nn = nn;

    if (nn < 3 || nn < MIN_ACCEPTED_SHARE * rrCount || sum < MIN_COVERAGE * windowMs)
        return r;

    float mean = sum / nn;
    float variance = 0;
    for (int i = 0; i < rrCount; i++)
        if (clean[i]) variance += (windowRR[i] - mean) * (windowRR[i] - mean);

    float sumSq = 0;
    int   pairs = 0;
    for (int i = 1; i < rrCount; i++)
    {
        if (clean[i] && clean[i - 1] && rrFollowsPrev[i])
        {
            float diff = windowRR[i] - windowRR[i - 1];
            sumSq += diff * diff;
            pairs++;
        }
    }
    if (pairs < 2) return r;

    r.sdnn  = sqrtf(variance / (nn - 1));
    r.rmssd = sqrtf(sumSq / pairs);
    r.valid = true;
    return r;
}

// ─────────────────────────────────────────────────────────────────────────────
// One-time setup — runs once when the device powers on or reboots.
// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200); // Open the serial console for debug messages

    // The default watchdog timeout (5 s) is shorter than the time it takes to
    // complete the encrypted connection to the MQTT broker.  Extending it to
    // 30 s prevents false "device frozen" reboots during that handshake.
    esp_task_wdt_init(30, true);

    setup_wifi();

    // Tell the secure connection layer which certificate authority to trust
    espClient.setCACert(ISRG_ROOT_X1);

    // Point the MQTT client at the broker and increase its send buffer so
    // our JSON payload fits in a single message
    mqttClient.setServer(MQTT_BROKER_IP, MQTT_BROKER_PORT);
    mqttClient.setBufferSize(512);

    Serial.println("Initializing MAX30102...");

    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
    {
        Serial.println("MAX30102 was not found. Check wiring/power.");
        while (1); // Halt — nothing useful can happen without the sensor
    }

    Serial.println("Place your index finger on the sensor.");

    // Configure the sensor LEDs:
    //   Red  (0x0A) — low power, used mainly for SpO2 (not read here)
    //   IR   (0x2F) — higher power, this is what beat detection uses
    //   Green (0)   — turned off, not needed for heart rate
    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeIR(0x2F);
    particleSensor.setPulseAmplitudeGreen(0);

    // Set up the thermistor: pin 32, 50 kΩ fixed resistor, 50 kΩ nominal NTC
    thermistor.begin(THERMISTOR_PIN, 50000, 50000);

    // Start the sample clock
    markSampleGap();
    lastDrainMs = millis();
}

// ─────────────────────────────────────────────────────────────────────────────
// Handle one IR sample from the sensor (called for every sample, in order).
// ─────────────────────────────────────────────────────────────────────────────
void processSample(long irValue)
{
    if (irValue < 50000)
    {
        // IR level below 50 000 means no finger is on the sensor.
        // Reset all beat tracking so stale data doesn't pollute the next reading.
        rateSpot = 0;
        rateCount = 0;
        beatAvg = 0;
        beatsPerMinute = 0;
        memset(rates, 0, sizeof(rates));

        // Keep the intervals already collected this window, but make sure the
        // first beat after the finger returns isn't measured across the gap.
        lastBeatSample   = -1;
        prevBeatAccepted = false;
    }
    else if (checkForBeat(irValue))
    {
        // A heartbeat was detected.  The filtered signal crossed zero between
        // the previous sample and this one; interpolate where.
        float prev = IR_AC_Signal_Previous;
        float cur  = IR_AC_Signal_Current;
        float frac = (cur > prev) ? -prev / (cur - prev) : 1.0f;
        double beatSample = (double)(sampleIndex - 1) + frac;

        bool  first = (lastBeatSample < 0); // no previous beat to measure from
        float delta = (float)((beatSample - lastBeatSample) * sampleMs);
        lastBeatSample = beatSample;

        if (first)
        {
            prevBeatAccepted = false;
        }
        else if (delta > 0)
        {
            beatsPerMinute = 60000.0f / delta;
            Serial.printf("[BEAT] delta=%.1fms raw=%.1f\n", delta, beatsPerMinute);

            // Only keep beats in a physiologically plausible range (45–150 BPM).
            // Values outside this are almost certainly noise or motion artefacts.
            if (beatsPerMinute > 45 && beatsPerMinute < 150)
            {
                rates[rateSpot] = (byte)beatsPerMinute;
                rateSpot = (rateSpot + 1) % RATE_SIZE; // Advance the circular buffer
                if (rateCount < RATE_SIZE) rateCount++;
                newBeatsThisCycle++;

                // Recalculate the running average BPM across all stored beats
                beatAvg = 0;
                for (byte i = 0; i < rateCount; i++)
                    beatAvg += rates[i];
                beatAvg /= rateCount;

                // Store the interval for this window's HRV, in time order
                if (rrCount < MAX_RR)
                {
                    windowRR[rrCount]      = delta;
                    rrFollowsPrev[rrCount] = prevBeatAccepted;
                    rrCount++;
                }
                prevBeatAccepted = true;
            }
            else
            {
                // Rejected as noise: the next interval doesn't follow a stored one
                prevBeatAccepted = false;
            }
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Main loop — runs repeatedly while the device is awake.
//
// Each pass through this loop:
//   1. Reads the infrared light level from the sensor.
//   2. If a fingertip is detected, checks whether a heartbeat just occurred.
//   3. Keeps the MQTT connection alive (reconnects if needed).
//   4. Every 60 seconds, sends all collected readings and then sleeps for 5 minutes.
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    // ── Read every new sample from the sensor ──────────────────────────────────
    // checkForBeat() needs an unbroken sample stream, and beat timing counts
    // samples, so drain the sensor's FIFO each pass instead of only reading the
    // latest value.  The library holds 4 samples per check(); if more arrived
    // (or the loop stalled long enough for the FIFO to overflow), samples were
    // lost and we mark a gap rather than time a beat across it.
    unsigned long nowMs = millis();
    unsigned long prevDrainMs = lastDrainMs;
    bool stalled = (nowMs - prevDrainMs > FIFO_GAP_MS);
    lastDrainMs = nowMs;

    uint16_t arrived = particleSensor.check();
    // 4 or more new samples overflow the library's 4-slot buffer, so some were lost
    if (stalled || arrived >= STORAGE_SIZE)
    {
        calibrateSampleClock(prevDrainMs); // the unbroken stretch ended at the last good drain
        gapsThisWindow++;
        while (particleSensor.available()) particleSensor.nextSample(); // discard partial data
        sampleIndex += arrived;
        markSampleGap();
    }
    else
    {
        while (particleSensor.available())
        {
            lastIR = particleSensor.getFIFOIR();
            particleSensor.nextSample();
            sampleIndex++;
            processSample(lastIR);
        }
    }
    long irValue = lastIR;

    // Print the raw IR sensor level every 500 ms so we can verify the sensor
    // is seeing pulsatile blood flow (the number should rise and fall with the pulse).
    static unsigned long lastIRDiag = 0;
    if (millis() - lastIRDiag >= 500)
    {
        Serial.printf("[IR] %ld\n", irValue);
        lastIRDiag = millis();
    }

    // ── Keep the MQTT connection alive ─────────────────────────────────────────
    // If WiFi is up but MQTT dropped, attempt a reconnect every 5 seconds.
    // If already connected, call mqttClient.loop() so it can handle keep-alive
    // ping/pong messages with the broker (required to stay connected).
    if (WiFi.status() == WL_CONNECTED && !mqttClient.connected())
    {
        static unsigned long lastReconnectAttempt = 0;
        long now = millis();
        if (now - lastReconnectAttempt > 5000)
        {
            lastReconnectAttempt = now;
            reconnect();
        }
    }
    else if (mqttClient.connected())
    {
        mqttClient.loop();
    }

    // ── Send readings and sleep ────────────────────────────────────────────────
    if (millis() - lastPublishTime >= PUBLISH_INTERVAL_MS)
    {
        // Gather all values for this upload window
        float tempC     = thermistor.read();          // °C
        float heartRate = beatAvg > 0 ? (float)beatAvg : beatsPerMinute;
        HrvResult hrv   = computeWindowHRV(PUBLISH_INTERVAL_MS);
        calibrateSampleClock(lastDrainMs); // before sleeping, so the sleep isn't counted

        // Print a summary line to the console
        Serial.printf("IR=%-7ld | BPM=%-6.1f | Avg BPM=%-4d | Temp=%.2f°C | sample=%.3fms | gaps=%u",
                      irValue, beatsPerMinute, beatAvg, tempC, sampleMs, gapsThisWindow);
        if (hrv.valid)
            Serial.printf(" | SDNN=%.1f | RMSSD=%.1f | NN=%d/%d", hrv.sdnn, hrv.rmssd, hrv.nn, rrCount);
        else
            Serial.printf(" | HRV=-- (quality check failed, NN=%d/%d)", hrv.nn, rrCount);
        if (irValue < 50000) Serial.print(" | No finger detected");
        Serial.println();

        // Send the readings if we have an MQTT connection
        if (!mqttClient.connected())
        {
            Serial.print("[MQTT] Not connected, state=");
            Serial.println(mqttClient.state());
        }
        else
        {
            bool ok = publishReading(heartRate, tempC, hrv.valid, hrv.sdnn, hrv.rmssd, newBeatsThisCycle);
            Serial.printf("[MQTT] Publish %s\n", ok ? "OK" : "FAILED");
        }

        // ── Enter light sleep ──────────────────────────────────────────────────
        // Close the network connection cleanly before sleeping so the server
        // doesn't think we've crashed, then put the chip into a low-power state.
        // Light sleep keeps WiFi alive internally, so reconnecting after wake is
        // much faster than a full reboot.
        Serial.println("---");
        Serial.printf("Entering light sleep for %llu seconds...\n", SLEEP_DURATION_US / 1000000ULL);
        Serial.flush(); // Make sure all console output is sent before sleeping

        mqttClient.disconnect(); // Politely close the MQTT session
        espClient.stop();        // Release the underlying network socket

        esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
        esp_light_sleep_start();
        // ── Device is asleep here ── the next line runs after the timer fires ──

        delay(500); // Give WiFi and the serial port a moment to fully wake up

        Serial.println("Woke from light sleep.");

        // Wait until we have a real IP address, not just a radio link.
        // Without a valid IP the server connection will fail immediately.
        unsigned long wifiWait = millis();
        while ((WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0))
               && millis() - wifiWait < 5000)
            delay(50);

        // A 5-minute sleep can outlast the access point's idle timeout, in which
        // case the AP has dropped us.  Ask the radio to re-associate and wait a
        // little longer before giving up on this upload.
        if (WiFi.status() != WL_CONNECTED)
        {
            Serial.println("[WiFi] Not connected after wake — reconnecting");
            WiFi.reconnect();
            wifiWait = millis();
            while ((WiFi.status() != WL_CONNECTED || WiFi.localIP() == IPAddress(0, 0, 0, 0))
                   && millis() - wifiWait < 10000)
                delay(100);
        }

        // The DNS resolver (which turns the broker hostname into an IP address)
        // needs a little extra time after the network comes back up.
        IPAddress brokerIP;
        unsigned long dnsWait = millis();
        while (!WiFi.hostByName(MQTT_BROKER_IP, brokerIP) && millis() - dnsWait < 5000)
            delay(100);
        Serial.printf("DNS resolved: %s\n", brokerIP.toString().c_str());

        // A further short pause lets the network routing finish settling.
        // Without it the encrypted connection often fails with a "connection
        // refused" error even though DNS already succeeded.
        delay(1500);

        // The last beat was before we slept, so the gap since then would include
        // the entire sleep period and produce a nonsense BPM on the first beat.
        // Forgetting it forces the algorithm to wait for two consecutive beats
        // before calculating a rate, which is the correct starting condition.
        lastBeatSample = -1;
        newBeatsThisCycle = 0;
        resetHrvWindow(); // the next reading's HRV uses only the next window's beats

        // Try to reconnect to MQTT up to 3 times.  The first attempt often
        // fails because the encrypted handshake needs the network fully stable.
        // Stopping the client before each attempt clears any half-open socket
        // left from the previous failure.
        for (int attempt = 1; attempt <= 3 && !mqttClient.connected(); attempt++)
        {
            Serial.printf("[MQTT] Connect attempt %d/3\n", attempt);
            espClient.stop();
            reconnect();
            if (!mqttClient.connected()) delay(2000);
        }

        // The sensor kept running and filling its internal buffer while we were
        // reconnecting (which can take several seconds).  Clearing the buffer
        // now means the beat-detection algorithm starts fresh on the next window
        // instead of processing a backlog of stale samples.
        particleSensor.clearFIFO();
        markSampleGap();           // restart beat timing on the fresh sample stream
        lastDrainMs = millis();
        gapsThisWindow = 0;

        // Mark the start of the next 60-second collection window
        lastPublishTime = millis();
    }
}
