// --- Blip counting for burst current measurement ---
#define BLIP_PIN 33 // Example pin for blip detection
#define BATTERY_PIN A13

#define ADC_SAMPLE_SIZE 500
float zero_current = -53.0; // current for no sun. used for mapping
volatile float blipCountMeasured = 0;

void IRAM_ATTR onBlip()
{
    blipCountMeasured++;
}

// Attach interrupt, count blips for a fixed window, then detach
float measureBlipCurrent(uint8_t pin, uint16_t windowMs)
{
    blipCountMeasured = 0;
    attachInterrupt(digitalPinToInterrupt(pin), onBlip, FALLING);
    unsigned long start = millis();
    while (millis() - start < windowMs)
    {
        // Wait for windowMs, ISR counts blips
    }
    detachInterrupt(digitalPinToInterrupt(pin));
    Serial.print(" Blip count: ");
    Serial.print(blipCountMeasured);
    return 1000 * blipCountMeasured / windowMs;
}
// Static solar panel monitor with INA219, Adafruit IO, and EPD display

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <AdafruitIO_WiFi.h>
#include <Adafruit_GFX.h>
// #include <Adafruit_EPD.h>
#include "Adafruit_ThinkInk.h"
#include "config.h" // Adafruit IO credentials
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
// ThinkInk_213_Tricolor_Z16 display(EPD_DC, EPD_RESET, EPD_CS, SRAM_CS, EPD_BUSY, EPD_SPI);

float busVoltage = 0;
float current = 0;
int ADC_voltage = 0;

void collectData()
{
    Serial.println("Collect and send data.");
    // Measure data from the INA219 and send to Adafruit IO using group publish
    // Measure battery with inbuilt ADC
    float batteryLevel = analogRead(BATTERY_PIN) / 4096.0 * 3.3 * 2 * 1.1;
    Serial.print("Battery Level: ");
    Serial.println(batteryLevel);
    Serial.print("ADC on CHRG:");
    // sample the ADC_voltage 500 times
    for (int i = 0; i < ADC_SAMPLE_SIZE; i++)
    {
        ADC_voltage += analogRead(BLIP_PIN);
    }
    ADC_voltage /= ADC_SAMPLE_SIZE;
    Serial.println(ADC_voltage);

    pinMode(BLIP_PIN, INPUT_PULLUP); // for the interrupts later

    if (!ina219.begin())
    {
        Serial.println("Failed to find INA219 chip");
        busVoltage = batteryLevel;
        current = -1;
    }
    else
    { // measure
        delay(200);
        busVoltage = ina219.getBusVoltage_V();
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
        current = 0;
        Serial.println(millis());
        // Sample the current 500 times and make an average
        for (int i = 0; i < ADC_SAMPLE_SIZE; i++)
        {
            current += ina219.getCurrent_mA();
        }
        current /= ADC_SAMPLE_SIZE;
    }
    // Serial.print(" Curr pre offset: ");
    // Serial.print(current);
    // current -= zero_current;//Adjust for quiescent current
    Serial.println(millis());

    Serial.print(" INA219 curr: ");
    Serial.print(current);

    Serial.print(" Voltage: ");
    Serial.print(busVoltage);

    Serial.print(" V");
    // Blip-based current measurement (100 ms window)
    blipCountMeasured = measureBlipCurrent(BLIP_PIN, 2000);
    float blipRatio = 0.0;
    if (blipCountMeasured > 0)
    {
        blipRatio = current / blipCountMeasured;
        Serial.print(" INA219 current / blip count ratio: ");
        Serial.print(blipRatio, 4);
    }
    else
    {
        Serial.println(" No blips detected.");
    }
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    pinMode(13, OUTPUT); // for blinking

    collectData();
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
    /*
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
  */

    // Print all values before publishing
    Serial.print("Publishing stat-bus-voltage: ");
    Serial.println(busVoltage);
    Serial.print("Publishing stat_current: ");
    Serial.println(current);
    Serial.print("Publishing stat_blip_count: ");
    Serial.println(blipCountMeasured);
    Serial.print("Publishing stat_adc_chrg: ");
    Serial.println(ADC_voltage);
    float blip_curr_ratio = (current != 0) ? blipCountMeasured / current : 0;
    float adc_curr_ratio = (current != 0) ? (4096 - ADC_voltage) / current : 0;
    Serial.print("Publishing stat_blip_curr_ratio: ");
    Serial.println(blip_curr_ratio);
    Serial.print("Publishing stat_adc_curr_ratio: ");
    Serial.println(adc_curr_ratio);

    io.run();
    // Publish to Adafruit IO with stat_ prefix
    group->set("stat-bus-voltage", busVoltage);
    group->set("stat_current", current);
    group->set("stat_blip_count", blipCountMeasured);
    group->set("stat_adc_chrg", ADC_voltage);
    group->set("stat_blip_curr_ratio", blip_curr_ratio);
    // group->set("stat_adc_curr_ratio", adc_curr_ratio);
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