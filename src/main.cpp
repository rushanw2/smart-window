#include <Arduino.h>
#include <cctype>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>

#if __has_include("config.h")
#include "config.h"
#else
#error "Missing include/config.h. Copy include/config.h.example to include/config.h and fill in your credentials."
#endif

namespace {

// Hardware configuration
constexpr uint8_t I2C_SDA_PIN = 21;
constexpr uint8_t I2C_SCL_PIN = 22;
constexpr uint8_t ALERT_LED_PIN = 2;

constexpr uint8_t OLED_WIDTH = 128;
constexpr uint8_t OLED_HEIGHT = 64;
constexpr int8_t OLED_RESET_PIN = -1;
constexpr uint8_t OLED_I2C_ADDR = 0x3C;

constexpr uint8_t BME280_PRIMARY_ADDR = 0x76;
constexpr uint8_t BME280_FALLBACK_ADDR = 0x77;

// Timing intervals (milliseconds)
constexpr unsigned long SENSOR_INTERVAL_MS = 5000;
constexpr unsigned long WEATHER_INTERVAL_MS = 300000;  // 5 minutes
constexpr unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;

// Hysteresis thresholds in degrees Fahrenheit:
// Alert turns ON when (indoor - outdoor) >= (DIFF_THRESHOLD_F + HYSTERESIS_F)
// Alert turns OFF when (indoor - outdoor) <= (DIFF_THRESHOLD_F - HYSTERESIS_F)
constexpr float DIFF_THRESHOLD_F = 2.0f;
constexpr float HYSTERESIS_F = 1.0f;
constexpr float MIN_INDOOR_TEMP_F = 62.0f;

Adafruit_BME280 bme;
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, OLED_RESET_PIN);

bool bmeReady = false;
bool oledReady = false;
bool hasIndoorReading = false;
bool hasOutdoorReading = false;
bool alertActive = false;

float indoorTempF = 0.0f;
float indoorHumidity = 0.0f;
float outdoorTempF = 0.0f;

unsigned long lastSensorReadMs = 0;
unsigned long lastWeatherFetchMs = 0;

String urlEncode(const char* raw) {
    String encoded;
    for (const char* p = raw; *p != '\0'; ++p) {
        const char c = *p;
        if (isalnum(static_cast<unsigned char>(c)) || c == '-' || c == '_' || c == '.' || c == '~' || c == ',') {
            encoded += c;
        } else if (c == ' ') {
            encoded += "%20";
        } else {
            char buf[4];
            snprintf(buf, sizeof(buf), "%%%02X", static_cast<unsigned char>(c));
            encoded += buf;
        }
    }
    return encoded;
}

bool initBME280() {
    if (bme.begin(BME280_PRIMARY_ADDR, &Wire)) {
        Serial.println(F("BME280 initialized at 0x76"));
        return true;
    }
    if (bme.begin(BME280_FALLBACK_ADDR, &Wire)) {
        Serial.println(F("BME280 initialized at 0x77 (fallback)"));
        return true;
    }
    Serial.println(F("ERROR: BME280 not found at 0x76 or 0x77"));
    return false;
}

bool initOLED() {
    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
        Serial.println(F("ERROR: SSD1306 OLED not found at 0x3C"));
        return false;
    }
    Serial.println(F("SSD1306 OLED initialized at 0x3C"));
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Smart Window Boot..."));
    display.display();
    return true;
}

bool ensureWiFiConnected() {
    if (WiFi.status() == WL_CONNECTED) {
        return true;
    }

    Serial.print(F("Connecting to Wi-Fi: "));
    Serial.println(WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    const unsigned long startMs = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < WIFI_CONNECT_TIMEOUT_MS) {
        delay(500);
        Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.print(F("Wi-Fi connected. IP: "));
        Serial.println(WiFi.localIP());
        return true;
    }

    Serial.println(F("WARNING: Wi-Fi connection timed out"));
    return false;
}

bool fetchOutdoorTempF(float& outTempF) {
    if (!ensureWiFiConnected()) {
        return false;
    }

    String url = F("https://api.openweathermap.org/data/2.5/weather?q=");
    url += urlEncode(CITY_QUERY);
    url += F("&appid=");
    url += OPENWEATHER_API_KEY;
    url += F("&units=imperial");

    WiFiClientSecure client;
    // Starter sketch uses setInsecure(); pin a root CA certificate for production use.
    client.setInsecure();

    HTTPClient http;
    if (!http.begin(client, url)) {
        Serial.println(F("ERROR: Failed to initialize HTTPClient"));
        return false;
    }

    const int httpCode = http.GET();
    if (httpCode != HTTP_CODE_OK) {
        Serial.printf("ERROR: OpenWeatherMap HTTP %d\n", httpCode);
        http.end();
        return false;
    }

    const String payload = http.getString();
    http.end();

    JsonDocument doc;
    const DeserializationError err = deserializeJson(doc, payload);
    if (err) {
        Serial.print(F("ERROR: JSON parse failed: "));
        Serial.println(err.c_str());
        return false;
    }

    if (doc["main"]["temp"].isNull()) {
        Serial.println(F("ERROR: Missing main.temp in OpenWeatherMap response"));
        return false;
    }

    outTempF = doc["main"]["temp"].as<float>();
    return true;
}

void readIndoorSensor() {
    if (!bmeReady) {
        return;
    }

    const float tempC = bme.readTemperature();
    const float hum = bme.readHumidity();
    if (!isnan(tempC)) {
        indoorTempF = tempC * 9.0f / 5.0f + 32.0f;
        indoorHumidity = isnan(hum) ? 0.0f : hum;
        hasIndoorReading = true;
    } else {
        Serial.println(F("WARNING: Failed to read temperature from BME280"));
    }
}

void updateHysteresisAlert() {
    if (!hasIndoorReading || !hasOutdoorReading) {
        alertActive = false;
        digitalWrite(ALERT_LED_PIN, LOW);
        return;
    }

    if (indoorTempF <= MIN_INDOOR_TEMP_F) {
        alertActive = false;
        digitalWrite(ALERT_LED_PIN, LOW);
        return;
    }

    const float diffF = indoorTempF - outdoorTempF;
    if (!alertActive &&
        indoorTempF >= (MIN_INDOOR_TEMP_F + HYSTERESIS_F) &&
        diffF >= (DIFF_THRESHOLD_F + HYSTERESIS_F)) {
        alertActive = true;
    } else if (alertActive && diffF <= (DIFF_THRESHOLD_F - HYSTERESIS_F)) {
        alertActive = false;
    }

    digitalWrite(ALERT_LED_PIN, alertActive ? HIGH : LOW);
}

void printStatus() {
    Serial.println(F("----- Smart Window Status -----"));
    if (hasIndoorReading) {
        Serial.printf("Indoor : %.1f F (%.1f%% RH)\n", indoorTempF, indoorHumidity);
    } else {
        Serial.println(F("Indoor : N/A (BME280 unavailable)"));
    }

    if (hasOutdoorReading) {
        Serial.printf("Outdoor: %.1f F\n", outdoorTempF);
    } else {
        Serial.println(F("Outdoor: N/A (waiting for API)"));
    }

    Serial.printf("Wi-Fi  : %s\n", WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected");
    Serial.printf("Alert  : %s (GPIO %u = %s)\n",
                  alertActive ? "OPEN WINDOW" : "CLOSE WINDOW",
                  ALERT_LED_PIN,
                  alertActive ? "HIGH" : "LOW");

    if (!oledReady) {
        return;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0, 0);
    display.println(F("SMART WINDOW"));

    display.setCursor(0, 14);
    if (hasIndoorReading) {
        display.printf("In : %.1f F  %.0f%%\n", indoorTempF, indoorHumidity);
    } else {
        display.println(F("In : ERR (BME280)"));
    }

    display.setCursor(0, 26);
    if (hasOutdoorReading) {
        display.printf("Out: %.1f F\n", outdoorTempF);
    } else {
        display.println(F("Out: --.- F"));
    }

    display.setCursor(0, 38);
    display.printf("WiFi: %s\n", WiFi.status() == WL_CONNECTED ? "OK" : "DISC");

    display.setCursor(0, 52);
    display.print(F("State: "));
    display.println(alertActive ? F("OPEN WINDOW") : F("CLOSE WINDOW"));

    display.display();
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println(F("Smart Window starting..."));

    pinMode(ALERT_LED_PIN, OUTPUT);
    digitalWrite(ALERT_LED_PIN, LOW);

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    oledReady = initOLED();
    bmeReady = initBME280();
    ensureWiFiConnected();

    readIndoorSensor();

    float fetchedTempF = 0.0f;
    if (fetchOutdoorTempF(fetchedTempF)) {
        outdoorTempF = fetchedTempF;
        hasOutdoorReading = true;
    }

    updateHysteresisAlert();
    printStatus();

    const unsigned long now = millis();
    lastSensorReadMs = now;
    lastWeatherFetchMs = now;
}

void loop() {
    const unsigned long now = millis();
    bool statusChanged = false;

    if (now - lastSensorReadMs >= SENSOR_INTERVAL_MS) {
        lastSensorReadMs = now;
        readIndoorSensor();
        statusChanged = true;
    }

    if (now - lastWeatherFetchMs >= WEATHER_INTERVAL_MS) {
        lastWeatherFetchMs = now;
        float fetchedTempF = 0.0f;
        if (fetchOutdoorTempF(fetchedTempF)) {
            outdoorTempF = fetchedTempF;
            hasOutdoorReading = true;
        }
        statusChanged = true;
    }

    if (statusChanged) {
        updateHysteresisAlert();
        printStatus();
    }
}
