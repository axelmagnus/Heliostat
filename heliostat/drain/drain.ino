#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

#include <Wire.h>
#include <Adafruit_MAX1704X.h>
#include <esp_sleep.h>
// SPI
#include <SPI.h>

// Use dedicated hardware SPI pins
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

Adafruit_MAX17048 maxlipo;

#define TARGET_VOLTAGE 3.6

// initialVoltage is now a global variable
float initialVoltage;

const char *ssid = "wrong_ssid";
const char *password = "wrong_password";

void showVoltage(float voltage, uint16_t color)
{
    tft.fillScreen(color);  
    tft.setTextColor(ST77XX_BLACK);
    tft.setTextSize(4);
    tft.setCursor(10, 32);

    tft.print(voltage, 2);
    tft.print(" V");
    // Add the run time in minutes
    tft.setCursor(10, 62);
    tft.setTextSize(2);
    tft.print("Run time:");
    tft.print((millis() / 60000), 0);
    tft.println(" min");
    // display the voltage drainage per minute since startup
    tft.setCursor(10, 90);
    tft.setTextSize(2);
    tft.println("Voltage drain: ");
    float minutes = millis() / 60000.0;
    float voltageDrainPerMin = (initialVoltage - voltage) / (minutes > 0 ? minutes : 1);
    tft.print(voltageDrainPerMin, 2);
    tft.print(" V/min");
}

void setup()
{
    Serial.begin(115200);
    // turn on backlite
    Serial.println("Serial ready");
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
    /*
        // turn on the TFT / I2C power supply
        pinMode(TFT_I2C_POWER, OUTPUT);
        digitalWrite(TFT_I2C_POWER, HIGH);
        delay(10);
    */
    // Initialize MAX17048
    while (!maxlipo.begin())
    {
        Serial.println("MAX17048 not found!");
    }

    Serial.print("Battery voltage pre: ");
    Serial.print(maxlipo.cellVoltage(), 3);
    float initialVoltage = maxlipo.cellVoltage();
    // initialize TFT
    tft.init(135, 240); // Init ST7789 240x135
    tft.setRotation(3);
}

void loop()
{
    float voltage = maxlipo.cellVoltage();
    showVoltage(voltage, ST77XX_WHITE);
    Serial.print("Battery voltage: ");
    Serial.println(voltage);
    delay(1000);

    if (voltage <= TARGET_VOLTAGE)
    {

        showVoltage(voltage, ST77XX_RED);
        delay(5000);
        esp_deep_sleep_start(); // Sleep forever
    }

    WiFi.begin(ssid, password);
    delay(5000); // Let WiFi try to connect and burn power
    WiFi.disconnect(true);
    delay(1000);
}