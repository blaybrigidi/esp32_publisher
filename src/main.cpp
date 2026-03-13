#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <ESP32_Thermistor.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "secrets.h"
#define THERMISTOR_PIN 32

// Publishing interval
const unsigned long PUBLISH_INTERVAL_MS = 1000;
unsigned long lastPublishTime = 0;

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

        // Mid-point reconnect nudge (~10 s in)
        if (!nudged && elapsed >= RECONNECT_MS)
        {
            WiFi.reconnect();
            nudged = true;
        }

        // Print a dot every 500 ms
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
        } // brief pause so Serial can flush
        ESP.restart();
    }

    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

bool publishReading(float hr, float temp, float hrv_sdnn, float hrv_rmssd)
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

    char buf[256];
    serializeJson(doc, buf);

    Serial.printf("[MQTT] Topic: %s  Payload: %s\n", topic, buf);
    return mqttClient.publish(topic, buf);
}

boolean reconnect()
{
    if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD))
    {
        Serial.println("MQTT connected");
    }
    else
    {
        Serial.print("MQTT failed, state=");
        Serial.println(mqttClient.state());
    }
    return mqttClient.connected();
}

// ---------------- Heart rate + HRV ----------------
const byte RATE_SIZE = 8;
byte rates[RATE_SIZE];
float rrIntervals[RATE_SIZE]; // R-R intervals in ms
byte rateSpot = 0;
byte rateCount = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

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

// ---------------- Serial printing ----------------
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 3000; // every 3s

void setup()
{
    Serial.begin(115200);

    setup_wifi();

    // Use insecure TLS for HiveMQ Cloud
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

    // Heart rate logic (unchanged from your improved version)
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

            if (beatsPerMinute > 45 && beatsPerMinute < 180)
            {
                rates[rateSpot] = (byte)beatsPerMinute;
                rrIntervals[rateSpot] = (float)delta; // store R-R interval (ms)
                rateSpot = (rateSpot + 1) % RATE_SIZE;
                if (rateCount < RATE_SIZE)
                    rateCount++;

                beatAvg = 0;
                for (byte i = 0; i < rateCount; i++)
                    beatAvg += rates[i];
                beatAvg /= rateCount;
            }
        }
    }

    // MQTT Reconnection
    if (WiFi.status() == WL_CONNECTED && !mqttClient.connected())
    {
        static unsigned long lastReconnectAttempt = 0;
        long now = millis();
        if (now - lastReconnectAttempt > 5000)
        {
            lastReconnectAttempt = now;
            if (reconnect())
                lastReconnectAttempt = 0;
        }
    }
    else if (mqttClient.connected())
    {
        mqttClient.loop();
    }

    // Publish vitals
    if (millis() - lastPublishTime >= PUBLISH_INTERVAL_MS)
    {
        lastPublishTime = millis();

        float tempC = thermistor.read();
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
            publishReading(heartRate, tempC, sdnn, rmssd);
        }

        Serial.println("---");
    }

    if (millis() - lastPrintTime >= PRINT_INTERVAL)
    {
        lastPrintTime = millis();
        Serial.print("IR=");
        Serial.print(irValue);
        Serial.print(" | BPM=");
        Serial.print(beatsPerMinute);
        Serial.print(" | Avg BPM=");
        Serial.print(beatAvg);
        Serial.print(" | Temp=");
        Serial.print(thermistor.read());
        Serial.println("°C");
    }
}