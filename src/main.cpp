// ─────────────────────────────────────────────────────────────────────────────
// DialOG — Wearable Vitals Monitor
//
// This device reads a patient's heart rate and finger temperature, then sends
// the results over WiFi to a cloud server every 15 seconds.  Between readings
// it goes into a light sleep to save battery while keeping WiFi alive.
// ─────────────────────────────────────────────────────────────────────────────

#include <Wire.h>
#include "MAX30105.h"       // Driver for the heart-rate / blood-oxygen sensor
#include "heartRate.h"      // Beat-detection algorithm for the sensor above
#include <ESP32_Thermistor.h>
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
// 15 seconds gives the beat-detection algorithm enough time to settle after
// waking from sleep.
const unsigned long PUBLISH_INTERVAL_MS = 15000;
unsigned long lastPublishTime = 0;

// How long the device sleeps between uploads (in microseconds).
// 10 seconds here is a short test value — change to (300ULL * 1000000ULL)
// for a 5-minute sleep in production.
#define SLEEP_DURATION_US (10ULL * 1000000ULL)

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
bool publishReading(float hr, float temp, float hrv_sdnn, float hrv_rmssd, byte beatCount)
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
    doc["hrv_sdnn"]    = hrv_sdnn;    // Heart-rate variability measure 1
    doc["hrv_rmssd"]   = hrv_rmssd;   // Heart-rate variability measure 2
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
// Heart-rate & heart-rate variability (HRV) state
//
// We keep a rolling buffer of the last 8 beats.  Each slot stores:
//   rates[]      — the BPM calculated for that beat
//   rrIntervals[]— the time gap between that beat and the previous one (ms)
//
// HRV (variability in those gaps) is an indicator of autonomic nervous system
// health.  Diabetic patients often show reduced HRV.
// ─────────────────────────────────────────────────────────────────────────────
const byte RATE_SIZE = 8;
byte  rates[RATE_SIZE];       // Rolling BPM readings
float rrIntervals[RATE_SIZE]; // Rolling beat-to-beat time gaps (ms)
byte  rateSpot  = 0;          // Write position in the circular buffer
byte  rateCount = 0;          // How many valid slots are filled so far
long  lastBeat  = 0;          // Timestamp of the most recent beat (ms)
float beatsPerMinute = 0;     // BPM from the most recent single beat gap
int   beatAvg   = 0;          // Average BPM across the last RATE_SIZE beats
byte  newBeatsThisCycle = 0;  // Valid beats collected since the last upload

// ─────────────────────────────────────────────────────────────────────────────
// SDNN — Standard Deviation of beat-to-beat intervals.
// Higher values generally mean a healthier, more adaptable heart rhythm.
// ─────────────────────────────────────────────────────────────────────────────
float computeSDNN(float *rr, byte count)
{
    if (count < 2) return 0.0f;

    // Step 1: find the average interval
    float mean = 0;
    for (byte i = 0; i < count; i++) mean += rr[i];
    mean /= count;

    // Step 2: measure how spread out the values are around that average
    float variance = 0;
    for (byte i = 0; i < count; i++)
        variance += (rr[i] - mean) * (rr[i] - mean);

    return sqrt(variance / count);
}

// ─────────────────────────────────────────────────────────────────────────────
// RMSSD — Root Mean Square of Successive Differences.
// Looks at how much consecutive beat gaps differ from each other.
// It is more sensitive to rapid changes in heart rhythm than SDNN.
// ─────────────────────────────────────────────────────────────────────────────
float computeRMSSD(float *rr, byte count)
{
    if (count < 2) return 0.0f;

    float sumSq = 0;
    for (byte i = 1; i < count; i++)
    {
        float diff = rr[i] - rr[i - 1]; // Gap between consecutive intervals
        sumSq += diff * diff;
    }
    return sqrt(sumSq / (count - 1));
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
}

// ─────────────────────────────────────────────────────────────────────────────
// Main loop — runs repeatedly while the device is awake.
//
// Each pass through this loop:
//   1. Reads the infrared light level from the sensor.
//   2. If a fingertip is detected, checks whether a heartbeat just occurred.
//   3. Keeps the MQTT connection alive (reconnects if needed).
//   4. Every 15 seconds, sends all collected readings and then sleeps briefly.
// ─────────────────────────────────────────────────────────────────────────────
void loop()
{
    long irValue = particleSensor.getIR();

    // Print the raw IR sensor level every 500 ms so we can verify the sensor
    // is seeing pulsatile blood flow (the number should rise and fall with the pulse).
    static unsigned long lastIRDiag = 0;
    if (millis() - lastIRDiag >= 500)
    {
        Serial.printf("[IR] %ld\n", irValue);
        lastIRDiag = millis();
    }

    if (irValue < 50000)
    {
        // IR level below 50 000 means no finger is on the sensor.
        // Reset all beat tracking so stale data doesn't pollute the next reading.
        rateSpot = 0;
        rateCount = 0;
        beatAvg = 0;
        beatsPerMinute = 0;
        memset(rates, 0, sizeof(rates));
        memset(rrIntervals, 0, sizeof(rrIntervals));
    }
    else if (checkForBeat(irValue))
    {
        // A heartbeat was detected.  Calculate BPM from the time since the last beat.
        long delta = millis() - lastBeat;
        lastBeat   = millis();

        if (delta > 0)
        {
            beatsPerMinute = 60.0 / (delta / 1000.0);
            Serial.printf("[BEAT] delta=%ldms raw=%.1f\n", delta, beatsPerMinute);

            // Only keep beats in a physiologically plausible range (45–150 BPM).
            // Values outside this are almost certainly noise or motion artefacts.
            if (beatsPerMinute > 45 && beatsPerMinute < 150)
            {
                rates[rateSpot]       = (byte)beatsPerMinute;
                rrIntervals[rateSpot] = (float)delta;
                rateSpot = (rateSpot + 1) % RATE_SIZE; // Advance the circular buffer
                if (rateCount < RATE_SIZE) rateCount++;
                newBeatsThisCycle++;

                // Recalculate the running average BPM across all stored beats
                beatAvg = 0;
                for (byte i = 0; i < rateCount; i++)
                    beatAvg += rates[i];
                beatAvg /= rateCount;
            }
        }
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
        float tempC     = thermistor.readTemperatureC();
        float heartRate = beatAvg > 0 ? (float)beatAvg : beatsPerMinute;
        float sdnn      = computeSDNN(rrIntervals, rateCount);
        float rmssd     = computeRMSSD(rrIntervals, rateCount);

        // Print a summary line to the console
        Serial.printf("IR=%-7ld | BPM=%-6.1f | Avg BPM=%-4d | Temp=%.2f°C | SDNN=%.1f | RMSSD=%.1f",
                      irValue, beatsPerMinute, beatAvg, tempC, sdnn, rmssd);
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
            bool ok = publishReading(heartRate, tempC, sdnn, rmssd, newBeatsThisCycle);
            Serial.printf("[MQTT] Publish %s\n", ok ? "OK" : "FAILED");
        }

        // ── Enter light sleep ──────────────────────────────────────────────────
        // Close the network connection cleanly before sleeping so the server
        // doesn't think we've crashed, then put the chip into a low-power state.
        // Light sleep keeps WiFi alive internally, so reconnecting after wake is
        // much faster than a full reboot.
        Serial.println("---");
        Serial.println("Entering light sleep for 10 seconds...");
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

        // lastBeat was set before we slept, so the gap since then would include
        // the entire sleep period and produce a nonsense BPM on the first beat.
        // Resetting it forces the algorithm to wait for two consecutive beats
        // before calculating a rate, which is the correct starting condition.
        lastBeat = 0;
        newBeatsThisCycle = 0;

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

        // Mark the start of the next 15-second collection window
        lastPublishTime = millis();
    }
}
