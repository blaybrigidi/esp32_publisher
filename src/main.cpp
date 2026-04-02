#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <ESP32_Thermistor.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_task_wdt.h"
#include "secrets.h"
#define THERMISTOR_PIN 32

// Active collection window before sleeping
const unsigned long PUBLISH_INTERVAL_MS = 15000; // 15 s — gives checkForBeat() time to re-stabilise after sleep gap
unsigned long lastPublishTime = 0;

// Light sleep duration — 10 s for testing, change to (300ULL * 1000000ULL) for production
#define SLEEP_DURATION_US (10ULL * 1000000ULL)

// ---------------- Sensors ----------------
MAX30105 particleSensor;
ESP32_Thermistor thermistor;

// ---------------- WiFi & MQTT ----------------
WiFiClientSecure espClient;
PubSubClient mqttClient(espClient);

void setup_wifi()
{
    const unsigned long TIMEOUT_MS = 20000;
    const unsigned long RECONNECT_MS = 10000;
    const unsigned long DOT_MS = 500;

    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long start = millis();
    unsigned long lastDot = start;
    bool nudged = false;

    while (WiFi.status() != WL_CONNECTED)
    {
        unsigned long elapsed = millis() - start;

        if (elapsed >= TIMEOUT_MS)
            break;

        if (!nudged && elapsed >= RECONNECT_MS)
        {
            WiFi.reconnect();
            nudged = true;
        }

        if (millis() - lastDot >= DOT_MS)
        {
            Serial.print(".");
            lastDot = millis();
        }
    }

    Serial.println();
    if (WiFi.status() != WL_CONNECTED)
    {
        int s = WiFi.status();
        Serial.print("WiFi failed, status=");
        Serial.print(s);
        Serial.print(" (");
        switch (s)
        {
        case WL_NO_SSID_AVAIL:
            Serial.print("SSID not found");
            break;
        case WL_CONNECT_FAILED:
            Serial.print("wrong password");
            break;
        case WL_DISCONNECTED:
            Serial.print("disconnected/auth");
            break;
        case WL_IDLE_STATUS:
            Serial.print("idle — timed out");
            break;
        default:
            Serial.print("unknown");
            break;
        }
        Serial.println(") — rebooting...");
        unsigned long wait = millis();
        while (millis() - wait < 3000)
        {
        }
        ESP.restart();
    }

    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

bool publishReading(float hr, float temp, float hrv_sdnn, float hrv_rmssd, byte beatCount)
{
    char topic[64];
    snprintf(topic, sizeof(topic), "vitals/%s", PATIENT_ID);

#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    StaticJsonDocument<256> doc;
#endif
    doc["patientId"] = PATIENT_ID;
    doc["deviceId"] = DEVICE_ID;
    doc["hr"] = hr;
    doc["temp"] = temp;
    doc["hrv_sdnn"] = hrv_sdnn;
    doc["hrv_rmssd"] = hrv_rmssd;
    doc["beat_count"] = beatCount;
    doc["esp_millis"] = millis(); // debug: device-side publish timestamp

    char buf[256];
    serializeJson(doc, buf);

    Serial.printf("[MQTT] Topic: %s  Payload: %s\n", topic, buf);
    return mqttClient.publish(topic, buf);
}

boolean reconnect()
{
    // Yield to FreeRTOS before blocking on TLS handshake — allows IDLE0 to run
    // and reset its watchdog subscription before we occupy CPU 0.
    delay(1);
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD))
        Serial.println("MQTT connected");
    else
    {
        Serial.print("MQTT failed, state=");
        Serial.println(mqttClient.state());
    }
    return mqttClient.connected();
}

// ---------------- Simulated temperature ----------------
// Thermistor returns NaN; simulate physiologically plausible peripheral
// (finger) temperatures, including diabetic patterns (poor circulation /
// peripheral neuropathy → lower temp, higher variance).
float simulateTemperature()
{
    bool diabeticPattern = (esp_random() % 10) < 3; // ~30% diabetic-like

    float base, range;
    if (diabeticPattern)
    {
        base = 29.5f; // lower peripheral temp due to poor circulation
        range = 5.0f; // ±2.5°C — more variable
    }
    else
    {
        base = 33.5f; // normal finger surface temperature
        range = 3.0f; // ±1.5°C
    }

    float noise = ((float)(esp_random() % 1000) / 1000.0f - 0.5f) * range;
    return base + noise;
}

// ---------------- Heart rate + HRV ----------------
const byte RATE_SIZE = 8;
byte rates[RATE_SIZE];
float rrIntervals[RATE_SIZE];
byte rateSpot = 0;
byte rateCount = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;
byte newBeatsThisCycle = 0; // valid beats collected since last publish

float computeSDNN(float *rr, byte count)
{
    if (count < 2)
        return 0.0f;
    float mean = 0;
    for (byte i = 0; i < count; i++)
        mean += rr[i];
    mean /= count;
    float variance = 0;
    for (byte i = 0; i < count; i++)
        variance += (rr[i] - mean) * (rr[i] - mean);
    return sqrt(variance / count);
}

float computeRMSSD(float *rr, byte count)
{
    if (count < 2)
        return 0.0f;
    float sumSq = 0;
    for (byte i = 1; i < count; i++)
    {
        float diff = rr[i] - rr[i - 1];
        sumSq += diff * diff;
    }
    return sqrt(sumSq / (count - 1));
}

void setup()
{
    Serial.begin(115200);

    // Extend task watchdog timeout to 30 s — the default 5 s is shorter than
    // the worst-case TLS handshake duration, causing spurious IDLE0 watchdog
    // crashes when mqttClient.connect() blocks CPU 0 during reconnect.
    esp_task_wdt_init(30, true);

    setup_wifi();

    espClient.setInsecure();
    mqttClient.setServer(MQTT_BROKER_IP, MQTT_BROKER_PORT);
    mqttClient.setBufferSize(512);

    Serial.println("Initializing MAX30102...");

    if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
    {
        Serial.println("MAX30102 was not found. Check wiring/power.");
        while (1)
            ;
    }

    Serial.println("Place your index finger on the sensor.");

    particleSensor.setup();
    particleSensor.setPulseAmplitudeRed(0x0A);
    particleSensor.setPulseAmplitudeIR(0x2F);
    particleSensor.setPulseAmplitudeGreen(0);

    thermistor.begin(THERMISTOR_PIN, 50000, 50000);
}

void loop()
{
    long irValue = particleSensor.getIR();

    // Diagnostic: print raw IR every 500ms so we can see pulsatile variation.
    // If IR is flat between [BEAT] events, checkForBeat() has no AC signal to work with.
    // If IR oscillates by thousands of counts, the algorithm should be detecting beats.
    static unsigned long lastIRDiag = 0;
    if (millis() - lastIRDiag >= 500)
    {
        Serial.printf("[IR] %ld\n", irValue);
        lastIRDiag = millis();
    }

    if (irValue < 50000)
    {
        rateSpot = 0;
        rateCount = 0;
        beatAvg = 0;
        beatsPerMinute = 0;
        memset(rates, 0, sizeof(rates));
        memset(rrIntervals, 0, sizeof(rrIntervals));
    }
    else if (checkForBeat(irValue))
    {
        long delta = millis() - lastBeat;
        lastBeat = millis();

        if (delta > 0)
        {
            beatsPerMinute = 60.0 / (delta / 1000.0);
            Serial.printf("[BEAT] delta=%ldms raw=%.1f\n", delta, beatsPerMinute);

            if (beatsPerMinute > 45 && beatsPerMinute < 150)
            {
                rates[rateSpot] = (byte)beatsPerMinute;
                rrIntervals[rateSpot] = (float)delta;
                rateSpot = (rateSpot + 1) % RATE_SIZE;
                if (rateCount < RATE_SIZE)
                    rateCount++;
                newBeatsThisCycle++;

                beatAvg = 0;
                for (byte i = 0; i < rateCount; i++)
                    beatAvg += rates[i];
                beatAvg /= rateCount;
            }
        }
    }

    // MQTT keepalive / reconnect fallback
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

    // Publish vitals then light sleep
    if (millis() - lastPublishTime >= PUBLISH_INTERVAL_MS)
    {
        float tempC = simulateTemperature();
        float heartRate = beatAvg > 0 ? (float)beatAvg : beatsPerMinute;
        float sdnn = computeSDNN(rrIntervals, rateCount);
        float rmssd = computeRMSSD(rrIntervals, rateCount);

        Serial.printf("IR=%-7ld | BPM=%-6.1f | Avg BPM=%-4d | Temp=%.2f°C | SDNN=%.1f | RMSSD=%.1f",
                      irValue, beatsPerMinute, beatAvg, tempC, sdnn, rmssd);
        if (irValue < 50000)
            Serial.print(" | No finger detected");
        Serial.println();

        if (!mqttClient.connected())
        {
            Serial.print("[MQTT] Not connected, state=");
            Serial.println(mqttClient.state());
        }
        else
        {
            publishReading(heartRate, tempC, sdnn, rmssd, newBeatsThisCycle);
        }

        Serial.println("---");
        Serial.println("Entering light sleep for 10 seconds...");
        Serial.flush();

        mqttClient.disconnect(); // close TLS session cleanly before sleep
        espClient.stop();        // release the TCP socket

        esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
        esp_light_sleep_start();

        // Execution resumes here after wake.
        // millis() continues during light sleep, so lastBeat is stale —
        // the first inter-beat interval after wake would include the full
        // sleep duration, producing an implausibly low BPM. Reset it so
        // the existing BPM filter seeds it cleanly on the next beat.
        delay(500); // let UART and WiFi stack fully settle after wake
        Serial.println("Woke from light sleep.");

        // Wait for WiFi to be ready before attempting TLS — rushing it causes errno 113
        unsigned long wifiWait = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - wifiWait < 3000)
            delay(50);

        lastBeat = 0;
        newBeatsThisCycle = 0;

        // Force immediate MQTT reconnect — static lastReconnectAttempt persists
        // across light sleep and would otherwise block attempts for up to 5 seconds
        reconnect();

        // Clear FIFO *after* reconnect — reconnect() blocks for several seconds
        // during TLS handshake, during which the sensor keeps running and overflows
        // the 32-sample FIFO again. Clearing after reconnect gives checkForBeat()
        // fresh samples from the moment the collection window starts.
        particleSensor.clearFIFO();
        lastPublishTime = millis(); // start window after reconnect + FIFO clear
    }
}
