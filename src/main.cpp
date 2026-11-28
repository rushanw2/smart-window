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

// Dual-color OLED bands: green (y = 0..15), physical gap (y = 16..17), blue (y = 18..63)
constexpr int16_t OLED_HEADER_Y = 4;
constexpr int16_t OLED_HERO_Y = 21;
constexpr int16_t OLED_DEG_CIRCLE_X = 76;
constexpr int16_t OLED_DEG_CIRCLE_Y = 22;
constexpr int16_t OLED_DEG_CIRCLE_R = 2;
constexpr int16_t OLED_HERO_UNIT_X = 80;
constexpr int16_t OLED_METRIC_X = 93;
constexpr int16_t OLED_OUTDOOR_Y = 19;
constexpr int16_t OLED_HUMIDITY_Y = 29;
constexpr int16_t OLED_WIFI_Y = 39;
constexpr int16_t OLED_HEARTBEAT_X = 123;
constexpr int16_t OLED_STATE_Y = 54;
constexpr int16_t OLED_STATE_BANNER_Y = 52;
constexpr int16_t OLED_STATE_BANNER_H = 12;

constexpr uint8_t BME280_PRIMARY_ADDR = 0x76;
constexpr uint8_t BME280_FALLBACK_ADDR = 0x77;

// Timing intervals (milliseconds)
constexpr unsigned long SENSOR_INTERVAL_MS = 5000;
constexpr unsigned long WEATHER_INTERVAL_MS = 300000;  // 5 minutes
constexpr unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000;
constexpr unsigned long ANIM_INTERVAL_MS = 500;

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
unsigned long lastAnimMs = 0;
uint8_t animFrame = 0;

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
    display.setTextWrap(false);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, OLED_HEADER_Y);
    display.print(F("Smart Window Boot..."));
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

void drawBannerText(int16_t rectY, int16_t rectH, int16_t textY, bool inverted, const char* text) {
    if (inverted) {
        display.fillRect(0, rectY, OLED_WIDTH, rectH, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
    } else {
        display.setTextColor(SSD1306_WHITE);
    }

    display.setCursor(0, textY);
    display.print(text);
    display.setTextColor(SSD1306_WHITE);
}

void renderOLED() {
    if (!oledReady) {
        return;
    }

    static constexpr char HEARTBEAT_FRAMES[4] = {'|', '/', '-', '\\'};

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextWrap(false);
    display.setTextColor(SSD1306_WHITE);

    // Top green band (y = 0..14)
    drawBannerText(0, 15, OLED_HEADER_Y, alertActive, "SMART WINDOW");

    // Blue zone hero indoor temperature (x = 0..89, y = 20..44)
    char heroTempBuf[8];
    if (!hasIndoorReading) {
        snprintf(heroTempBuf, sizeof(heroTempBuf), "--.-");
    } else if (indoorTempF >= 100.0f || indoorTempF <= -10.0f) {
        snprintf(heroTempBuf, sizeof(heroTempBuf), "%.0f", indoorTempF);
    } else {
        snprintf(heroTempBuf, sizeof(heroTempBuf), "%.1f", indoorTempF);
    }

    display.setTextSize(3);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, OLED_HERO_Y);
    display.print(heroTempBuf);

    display.drawCircle(OLED_DEG_CIRCLE_X, OLED_DEG_CIRCLE_Y, OLED_DEG_CIRCLE_R, SSD1306_WHITE);
    display.setTextSize(2);
    display.setCursor(OLED_HERO_UNIT_X, OLED_HERO_Y);
    display.print('F');

    // Reset before right-hand metrics (x = 93..127)
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(OLED_METRIC_X, OLED_OUTDOOR_Y);
    if (!hasOutdoorReading) {
        display.print(F("Out:--"));
    } else if (outdoorTempF >= 99.5f || outdoorTempF <= -9.5f) {
        display.printf("O:%.0f", outdoorTempF);
    } else {
        display.printf("Out:%.0f", outdoorTempF);
    }

    display.setCursor(OLED_METRIC_X, OLED_HUMIDITY_Y);
    if (!hasIndoorReading) {
        display.print(F("RH:--%"));
    } else if (indoorHumidity >= 99.5f) {
        display.print(F("H:100%"));
    } else {
        display.printf("RH:%.0f%%", indoorHumidity);
    }

    display.setCursor(OLED_METRIC_X, OLED_WIFI_Y);
    display.print(WiFi.status() == WL_CONNECTED ? F("WF:OK") : F("WF:--"));
    display.setCursor(OLED_HEARTBEAT_X, OLED_WIFI_Y);
    display.print(HEARTBEAT_FRAMES[animFrame % 4U]);

    // Reset before bottom blue banner (y = 52..63)
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    drawBannerText(OLED_STATE_BANNER_Y,
                   OLED_STATE_BANNER_H,
                   OLED_STATE_Y,
                   alertActive,
                   alertActive ? "State: OPEN WINDOW" : "State: CLOSE WINDOW");

    display.display();
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

    renderOLED();
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
    lastAnimMs = now;
}

void loop() {
    const unsigned long now = millis();
    bool statusChanged = false;
    bool animChanged = false;

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

    if (now - lastAnimMs >= ANIM_INTERVAL_MS) {
        lastAnimMs = now;
        animFrame = static_cast<uint8_t>((animFrame + 1U) % 4U);
        animChanged = true;
    }

    if (statusChanged) {
        updateHysteresisAlert();
        printStatus();
    } else if (animChanged) {
        renderOLED();
    }
}
