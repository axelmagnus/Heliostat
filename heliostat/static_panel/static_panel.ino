// Static solar panel monitor with INA219, Adafruit IO, and EPD display 

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <AdafruitIO_WiFi.h>
#include <Adafruit_GFX.h>
// #include <Adafruit_EPD.h>
#include "Adafruit_ThinkInk.h"
#include "config.h"  // Adafruit IO credentials
// #include <WiFiClientSecure.h>

#define SLEEP_MINUTES 10
#define BATT_SAMPLE_SIZE 500

#define EPD_DC 33
#define EPD_CS 15
#define EPD_BUSY -1 // can set to -1 to not use a pin (will wait a fixed delay)
#define SRAM_CS 32
#define EPD_RESET -1 // can set to -1 and share with microcontroller Reset!
#define EPD_SPI &SPI // primary SPI

Adafruit_INA219 ina219;
AdafruitIO_Group *group = io.group("heliostat");
ThinkInk_213_Tricolor_Z16 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

float busVoltage = 0;
float current = 0;

void setup()
{
    Serial.begin(115200);
    delay(500);
    pinMode(13, OUTPUT); // for blinking

    // INA219 setu

    if (!ina219.begin())
    {
        Serial.println("ERROR: Failed to find INA219 chip. Halting.");
    }
    current = 0;

    for (int i = 0; i < BATT_SAMPLE_SIZE; i++)
    {
        float mA = ina219.getCurrent_mA();
        current += mA;
    }
    current /= BATT_SAMPLE_SIZE;
    Serial.println(current);
    Serial.println(ina219.getBusVoltage_V());
    //if voltage is below 3.3 V go to sleep again
    if (ina219.getBusVoltage_V() < 3.3)
    {
        Serial.println("Bus voltage low, going to sleep.");

        // Blink LED 3 times
        for (int i = 0; i < 3; i++)
        {
            digitalWrite(13, HIGH);
            delay(100);
            digitalWrite(13, LOW);
            delay(100);
        }
        esp_deep_sleep(SLEEP_MINUTES * 60 * 1000000ULL);
    }

    // Adafruit IO setup
    Serial.println(WIFI_SSID);
    io.connect();
    int aioTries = 0;
    while (io.status() < AIO_CONNECTED && aioTries < 20)
    {
        Serial.println(io.statusText());
        digitalWrite(13, aioTries % 2);
        delay(500);
        aioTries++;
    }

    display.begin(THINKINK_TRICOLOR);
    Serial.println("Started EPD");

    delay(200);
    // Display voltage and current in large font
    display.clearBuffer();
    display.setTextSize(3);
    display.setCursor(5, 10);
    display.setTextColor(EPD_BLACK);
    display.print(ina219.getBusVoltage_V(), 2);
    display.print(" V");
    display.setCursor(5, 20);
    display.print(current, 1);
    display.print(" mA");

    // Download and draw historic graphs
    // bool graphError = !drawHistoricGraphs();

    display.display();
    io.run();
    // Publish to Adafruit IO with stat_ prefix
    bool aioError = false;
    group->set("stat-bus-voltage", ina219.getBusVoltage_V());
    group->set("stat_current", current);
    group->save();
    delay(2000);

    // Sleep for 10 minutes only if no errors
    Serial.print("Sleeping ");
    Serial.println(SLEEP_MINUTES);
    esp_deep_sleep(SLEEP_MINUTES * 60 * 1000000ULL);
}

void loop()
{
}