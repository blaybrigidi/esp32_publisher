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
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(WIFI_SSID);

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

bool publishVital(const char *type, float value, const char *unit)
{
    char topic[80];
    snprintf(topic, sizeof(topic), "vitals/%s/%s", PATIENT_ID, type);

#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    StaticJsonDocument<256> doc;
#endif
    doc["patientId"] = PATIENT_ID;
    doc["type"] = type;
    doc["value"] = value;
    doc["unit"] = unit;
    doc["deviceId"] = DEVICE_ID;

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

// ---------------- Heart rate ----------------
const byte RATE_SIZE = 8;
byte rates[RATE_SIZE];
byte rateSpot = 0;
byte rateCount = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;

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
                rates[rateSpot++] = (byte)beatsPerMinute;
                rateSpot %= RATE_SIZE;
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

        Serial.printf("IR=%-7ld | BPM=%-6.1f | Avg BPM=%-4d | Temp=%.2f°C", irValue, beatsPerMinute, beatAvg, tempC);
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
            publishVital("HEART_RATE", heartRate, "bpm");
            publishVital("TEMPERATURE", tempC, "C");
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