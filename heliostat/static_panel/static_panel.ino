// Static solar panel monitor with INA219, SHT41, TFT display for ESP32-S2
// Wakes every 10 minutes to send data to Adafruit IO
// Button press on D2 wakes device to display data without internet connection

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SHT4x.h>
#include <AdafruitIO_WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "config.h" // Adafruit IO credentials
#include <esp_sleep.h>
#include <driver/rtc_io.h>

#define SLEEP_MINUTES 10
#define ADC_SAMPLE_SIZE 500
#define BUTTON_PIN GPIO_NUM_2  // D2 button for wake-up

// ESP32-S2 TFT Feather pins
#define TFT_CS         7
#define TFT_DC         39
#define TFT_RST        40
#define TFT_BACKLIGHT  45

// Teal color scheme
#define COLOR_DARK_TEAL   0x0410   // Dark teal background
#define COLOR_MID_TEAL    0x4E9C   // Medium teal for boxes
#define COLOR_LIGHT_TEAL  0xAF3D   // Light teal for text

Adafruit_INA219 ina219;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
AdafruitIO_Group *group = io.group("heliostat");

// RTC memory to persist data across deep sleep
RTC_DATA_ATTR float rtc_busVoltage = 0;
RTC_DATA_ATTR float rtc_current = 0;
RTC_DATA_ATTR float rtc_temperature = 0;
RTC_DATA_ATTR float rtc_humidity = 0;
RTC_DATA_ATTR uint64_t rtc_sleep_start = 0;
RTC_DATA_ATTR uint64_t rtc_total_sleep_time = 0;

float busVoltage = 0;
float current = 0;
float temperature = 0;
float humidity = 0;

void collectData()
{
    Serial.println("Collecting sensor data...");
    
    // Measure INA219 voltage (quick reading for display)
    if (!ina219.begin())
    {
        Serial.println("Failed to find INA219 chip");
        busVoltage = rtc_busVoltage; // Use last known value
        current = rtc_current;
    }
    else
    {
        busVoltage = ina219.getBusVoltage_V();
        
        // Check for low voltage condition
        if (busVoltage < 3.3)
        {
            Serial.println("Bus voltage critically low, going to sleep.");
            esp_deep_sleep(SLEEP_MINUTES * 60 * 1000000ULL);
        }
        
        
        Serial.print("Quick voltage: ");
        Serial.print(busVoltage);
        Serial.print(" V, current: ");
        Serial.print(current);
        Serial.println(" mA");
    }
    
    // Measure SHT41 temperature and humidity
    if (!sht4.begin())
    {
        Serial.println("Failed to find SHT41 chip");
        temperature = rtc_temperature; // Use last known value
        humidity = rtc_humidity;
    }
    else
    {
        sensors_event_t humidity_event, temp_event;
        sht4.getEvent(&humidity_event, &temp_event);
        temperature = temp_event.temperature;
        humidity = humidity_event.relative_humidity;
        
        Serial.print("Temperature: ");
        Serial.print(temperature);
        Serial.print(" °C, Humidity: ");
        Serial.print(humidity);
        Serial.println(" %");
    }
    
    // Store quick readings in RTC memory
    rtc_busVoltage = busVoltage;
    rtc_current = current;
    rtc_temperature = temperature;
    rtc_humidity = humidity;
}

void collectDataAccurate()
{
    Serial.println("Collecting accurate sensor data (500 samples)...");
    
    // Accurate 500-sample current measurement
    if (ina219.begin())
    {
        current = 0;
        unsigned long start = millis();
        for (int i = 0; i < ADC_SAMPLE_SIZE; i++)
        {
            current += ina219.getCurrent_mA();
        }
        current /= ADC_SAMPLE_SIZE;
        Serial.print("Accurate current measurement took ");
        Serial.print(millis() - start);
        Serial.print(" ms: ");
        Serial.print(current);
        Serial.println(" mA");
        
        // Update RTC memory with accurate reading
        rtc_current = current;
    }
}

void displayData()
{
    // Turn on backlight
    pinMode(TFT_BACKLIGHT, OUTPUT);
    digitalWrite(TFT_BACKLIGHT, HIGH);
    
    // Initialize TFT
    tft.init(135, 240);
    tft.setRotation(3); // Landscape orientation
    tft.fillScreen(COLOR_DARK_TEAL);
    
    // Draw rounded rectangle containers
    tft.fillRoundRect(5, 5, 110, 55, 8, COLOR_MID_TEAL);
    tft.fillRoundRect(125, 5, 110, 55, 8, COLOR_MID_TEAL);
    tft.fillRoundRect(5, 70, 110, 55, 8, COLOR_MID_TEAL);
    tft.fillRoundRect(125, 70, 110, 55, 8, COLOR_MID_TEAL);
    
    // Set text color
    tft.setTextColor(COLOR_LIGHT_TEAL);
    
    // Display Voltage
    tft.setTextSize(2);
    tft.setCursor(15, 15);
    tft.print("Voltage");
    tft.setTextSize(3);
    tft.setCursor(15, 35);
    tft.print(busVoltage, 2);
    tft.setTextSize(2);
    tft.print("V");
    
    // Display Current
    tft.setTextSize(2);
    tft.setCursor(135, 15);
    tft.print("Current");
    tft.setTextSize(3);
    tft.setCursor(135, 35);
    tft.print(current, 0);
    tft.setTextSize(2);
    tft.print("mA");
    
    // Display Temperature
    tft.setTextSize(2);
    tft.setCursor(15, 80);
    tft.print("Temp");
    tft.setTextSize(3);
    tft.setCursor(15, 100);
    tft.print(temperature, 1);
    tft.setTextSize(2);
    tft.print("C");
    
    // Display Humidity
    tft.setTextSize(2);
    tft.setCursor(135, 80);
    tft.print("Humidity");
    tft.setTextSize(3);
    tft.setCursor(135, 100);
    tft.print(humidity, 0);
    tft.setTextSize(2);
    tft.print("%");
    
    Serial.println("Display updated");
}

void sendDataToAIO()
{
    Serial.println("Connecting to Adafruit IO...");
    Serial.println(WIFI_SSID);
    io.connect();
    
    int aioTries = 0;
    while (io.status() < AIO_CONNECTED && aioTries < 20)
    {
        Serial.println(io.statusText());
        delay(500);
        aioTries++;
    }
    
    if (io.status() < AIO_CONNECTED)
    {
        Serial.println("Failed to connect to AIO, skipping data send");
        Serial.println("Sleeping for 10 minutes...");
        esp_sleep_enable_timer_wakeup(SLEEP_MINUTES * 60 * 1000000ULL);
        esp_deep_sleep_start();
    }
    
    Serial.println("Connected to AIO");
    
    // Perform accurate current measurement for cloud reporting
    collectDataAccurate();
    
    // Publish to Adafruit IO with stat_ prefix
    Serial.print("Publishing stat_bus_voltage: ");
    Serial.println(busVoltage);
    Serial.print("Publishing stat_current: ");
    Serial.println(current);
    Serial.print("Publishing stat_temperature: ");
    Serial.println(temperature);
    Serial.print("Publishing stat_humidity: ");
    Serial.println(humidity);
    
    group->set("stat_bus_voltage", busVoltage);
    group->set("stat_current", current);
    group->set("stat_temperature", temperature);
    group->set("stat_humidity", humidity);
    group->save();
    
    unsigned long startTime = millis();
    unsigned long runDuration = 5000; // Run io.run() for 5 seconds

    while (millis() - startTime < runDuration) {
      io.run();
      delay(10); // Small delay to avoid blocking other tasks
    }

    Serial.println("Data published successfully");
}

void setup()
{
    Serial.begin(115200);
    delay(500);
    
    // Determine wake-up reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    Serial.print("Wake-up cause: ");
    Serial.println(wakeup_reason);
    
    // Collect initial sensor data (quick readings)
    collectData();
    
    if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
    {
        // Button press wake-up - display only, no internet
        Serial.println("Button wake-up: Display data without internet");
        displayData();
        delay(2000); // Show display for 2 seconds
        
        // Turn off backlight
        digitalWrite(TFT_BACKLIGHT, LOW);
        
        // Calculate remaining sleep time
        uint64_t elapsed_us = esp_timer_get_time() - rtc_sleep_start;
        uint64_t sleep_target_us = SLEEP_MINUTES * 60 * 1000000ULL;
        uint64_t remaining_sleep_us = 0;
        
        if (elapsed_us < sleep_target_us)
        {
            remaining_sleep_us = sleep_target_us - elapsed_us;
        }
        else
        {
            remaining_sleep_us = sleep_target_us; // Full sleep cycle if exceeded
        }
        
        Serial.print("Sleeping for remaining time: ");
        Serial.print(remaining_sleep_us / 1000000);
        Serial.println(" seconds");
        
        // Start accurate current measurement in background before sleep
        collectDataAccurate();
        
        // Configure wake-up sources
        esp_sleep_enable_ext0_wakeup(BUTTON_PIN, 0); // Wake on button press (LOW)
        esp_sleep_enable_timer_wakeup(remaining_sleep_us);
        
        // Store sleep start time
        rtc_sleep_start = esp_timer_get_time();
        
        esp_deep_sleep_start();
    }
    else
    {
        // Timer wake-up or reset - display AND send data
        Serial.println("Timer wake-up or reset: Display and send data");
        
        // Display data on TFT
        displayData();
        
        // Send data to Adafruit IO (includes accurate measurement)
        sendDataToAIO();
        
        // Show display for 2 seconds after data is sent
        delay(2000);
        
        // Turn off backlight
        digitalWrite(TFT_BACKLIGHT, LOW);
        
        Serial.print("Sleeping for ");
        Serial.print(SLEEP_MINUTES);
        Serial.println(" minutes");
        
        // Configure wake-up sources
        esp_sleep_enable_ext0_wakeup(BUTTON_PIN, 0); // Wake on button press (LOW)
        esp_sleep_enable_timer_wakeup(SLEEP_MINUTES * 60 * 1000000ULL);
        
        // Store sleep start time
        rtc_sleep_start = esp_timer_get_time();
        
        esp_deep_sleep_start();
    }
}

void loop()
{
    // Not used - everything happens in setup() before deep sleep
}