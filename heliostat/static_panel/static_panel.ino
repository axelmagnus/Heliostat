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
#include <sys/time.h>

#define SLEEP_MINUTES 10
#define ADC_SAMPLE_SIZE 500
#define BUTTON_PIN T12       // A1 / TOUCH9 for capacitive touch wake-up
#define TOUCH_THRESHOLD 8000 // Touch sensitivity threshold (lower = more sensitive)

// ESP32-S2 TFT Feather pins
#define TFT_CS 7
#define TFT_DC 39
#define TFT_RST 40
#define TFT_BACKLIGHT 45
#define TFT_I2C_POWER 21 // I2C bus power control

// Teal color scheme
#define COLOR_DARK_TEAL 0x0410  // Dark teal background
#define COLOR_MID_TEAL 0x4E9C   // Medium teal for boxes
#define COLOR_LIGHT_TEAL 0xAF3D // Light teal for text

Adafruit_INA219 ina219;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
AdafruitIO_Group *group = io.group("heliostat");

// RTC memory to persist data across deep sleep
RTC_DATA_ATTR float rtc_busVoltage = 3.33;
RTC_DATA_ATTR float rtc_current = 0;
RTC_DATA_ATTR float rtc_temperature = 0;
RTC_DATA_ATTR float rtc_humidity = 0;
RTC_DATA_ATTR uint64_t rtc_remaining_sleep_us = 0;          // Remaining time until next 10-min timer wake
RTC_DATA_ATTR struct timeval rtc_sleep_enter_time = {0, 0}; // Time when we last entered deep sleep

float busVoltage = 0;
float current = 0;
float temperature = 0;
float humidity = 0;
bool inaReady = false;

void initSensors()
{
    Serial.println("Initializing sensors...");
    // Init SHt41
    if (!sht4.begin())
    {
        Serial.println("Failed to find SHT41 chip");
    }
    // INA219 Current Sensor
    if (!inaReady)
    {
        Serial.println("Trying INA219 begin()...");
        if (ina219.begin())
        {
            inaReady = true;
            Serial.println("INA219 initialized");
        }
        else
        {
            Serial.println("INA219 not found at expected address (0x40). Check wiring/power.");
        }
    }
}

void collectData()
{
    Serial.println("Collecting sensor data...");

    // Measure bus voltage using INA219
    if (inaReady)
    {
        busVoltage = ina219.getBusVoltage_V();
        current = ina219.getCurrent_mA();
    }
    else
    {
        Serial.println("INA219 not initialized - using RTC cached voltage");
        busVoltage = rtc_busVoltage;
        current = rtc_current;
    }

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
    if (inaReady)
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

    // Display Temperature (top left)
    tft.setTextSize(2);
    tft.setCursor(15, 15);
    tft.print("Temp");
    tft.setTextSize(3);
    tft.setCursor(15, 35);
    tft.print(temperature, 1);
    tft.setTextSize(2);
    tft.print(" C");

    // Display Voltage (top right)
    tft.setTextSize(2);
    tft.setCursor(135, 15);
    tft.print("Voltage");
    tft.setTextSize(3);
    tft.setCursor(135, 35);
    tft.print(busVoltage, 2);
    tft.setTextSize(2);
    tft.print(" V");

    // Display Current (bottom left)
    tft.setTextSize(2);
    tft.setCursor(15, 80);
    tft.print("Current");
    tft.setTextSize(3);
    tft.setCursor(15, 100);
    tft.print(current, 0);
    tft.setTextSize(2);
    tft.print(" mA");

    // Display Humidity (bottom right)
    tft.setTextSize(2);
    tft.setCursor(135, 80);
    tft.print("Humidity");
    tft.setTextSize(3);
    tft.setCursor(135, 100);
    tft.print(humidity, 0);
    tft.setTextSize(2);
    tft.print(" %");

    Serial.println("Display updated");
}

void updateDisplay()
{
    // Only update the current value without redrawing entire screen
    // Clear the current display area by redrawing the box
    tft.fillRoundRect(125, 5, 110, 55, 8, COLOR_MID_TEAL);

    tft.setTextColor(COLOR_LIGHT_TEAL);

    // Redraw Current
    tft.setTextSize(2);
    tft.setCursor(135, 15);
    tft.print("Current");
    tft.setTextSize(3);
    tft.setCursor(135, 35);
    tft.print(current, 0);
    tft.setTextSize(2);
    tft.print("mA");

    Serial.println("Display current updated");
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
    collectData(); // Collect fresh data before sending
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
    // battery percent removed
    group->set("stat_current", current);
    group->set("stat_temperature", temperature);
    group->set("stat_humidity", humidity);
    group->save();

    unsigned long startTime = millis();
    unsigned long runDuration = 5000; // Run io.run() for 5 seconds

    while (millis() - startTime < runDuration)
    {
        io.run();
        if (io.status() == AIO_CONNECTED)
        {
            Serial.println("Data successfully sent to Adafruit IO, exiting early.");
            break;
        }
        else
        {
            Serial.println("sending...");
        }
        delay(10); // Small delay to avoid blocking other tasks
    }
    Serial.println("Data published successfully");
}

void setup()
{
    Serial.begin(115200);
    // start the I2c bus writing TFT_I2C power on
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(300); // Wait for power to stabilize
    // while (!Serial){};
    //  Initialize I2C bus (default speed) and sensors
    initSensors();
    // Determine wake-up reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    Serial.print("Wake-up cause: ");
    Serial.println(wakeup_reason);
    // Collect initial sensor data (quick readings)
    // collectData();
    Serial.println(millis());

    // Configure touch pad for capacitive touch wake-up
    // GPIO 12 is TOUCH9 on ESP32-S2
    touchAttachInterrupt(
        BUTTON_PIN, []() {}, TOUCH_THRESHOLD);

    if (wakeup_reason == ESP_SLEEP_WAKEUP_TOUCHPAD)
    { // Touch wake-up - display only, no internet
        Serial.println("Touch wake-up: Display data without internet");

        // Show splash screen immediately
        pinMode(TFT_BACKLIGHT, OUTPUT);
        digitalWrite(TFT_BACKLIGHT, HIGH);
        tft.init(135, 240);
        tft.setRotation(3);
        tft.fillScreen(COLOR_DARK_TEAL);
        tft.setTextColor(COLOR_LIGHT_TEAL);
        tft.setTextSize(3);
        tft.setCursor(60, 55);
        tft.print("AXEL");

        Serial.println(millis());
        collectData();
        collectDataAccurate(); // Accurate measurement takes time
        displayData();         // Show quick data from RTC first
        // updateDisplay(); // Update only the current value
        delay(5000); // Show display for 2 seconds

        // Turn off backlight
        digitalWrite(TFT_BACKLIGHT, LOW);

        // Compute elapsed sleep time since last deep sleep entry
        struct timeval now;
        gettimeofday(&now, NULL);
        uint64_t elapsed_us = 0;
        if (rtc_sleep_enter_time.tv_sec != 0)
        {
            elapsed_us = (uint64_t)(now.tv_sec - rtc_sleep_enter_time.tv_sec) * 1000000ULL +
                         (uint64_t)(now.tv_usec - rtc_sleep_enter_time.tv_usec);
        }

        if (rtc_remaining_sleep_us == 0)
        {
            rtc_remaining_sleep_us = SLEEP_MINUTES * 60 * 1000000ULL;
        }

        // Subtract elapsed time (from timer sleep) from remaining window
        if (elapsed_us > 0 && elapsed_us < rtc_remaining_sleep_us)
        {
            rtc_remaining_sleep_us -= elapsed_us;
        }

        uint64_t remaining_sleep_us = rtc_remaining_sleep_us;

        Serial.print("Sleeping for remaining time: ");
        Serial.print(remaining_sleep_us / 1000000);
        Serial.println(" seconds");

        // Configure wake-up sources
        touchSleepWakeUpEnable(BUTTON_PIN, TOUCH_THRESHOLD); // Enable touch wake-up on GPIO 18 (TOUCH9)
        esp_sleep_enable_timer_wakeup(remaining_sleep_us);

        // Record deep sleep entry time for next wake
        gettimeofday(&rtc_sleep_enter_time, NULL);

        esp_deep_sleep_start();
    } // touch wake-up

    if (wakeup_reason == ESP_SLEEP_WAKEUP_UNDEFINED)
    { // Reset wake-up - display AND send data
        Serial.println("Reset wake-up: Display and send data");

        // Reset sleep cycle tracking for a fresh 10-minute window
        rtc_remaining_sleep_us = SLEEP_MINUTES * 60 * 1000000ULL;
        rtc_sleep_enter_time = {0, 0};

        // Show splash screen immediately
        pinMode(TFT_BACKLIGHT, OUTPUT);
        digitalWrite(TFT_BACKLIGHT, HIGH);
        tft.init(135, 240);
        tft.setRotation(3);
        tft.fillScreen(COLOR_DARK_TEAL);
        tft.setTextColor(COLOR_LIGHT_TEAL);
        tft.setTextSize(4);
        tft.setCursor(50, 55);
        tft.print("Axel");

        collectData();
        //  Collect accurate data after initial display
        collectDataAccurate();
        displayData(); // Show  data
        // updateDisplay(); // Update only the current value
        delay(3000); // Show display for 2 seconds

        // Turn off backlight
        digitalWrite(TFT_BACKLIGHT, LOW);

        // Send data to Adafruit IO (includes accurate measurement)
        sendDataToAIO();

        Serial.print("Sleeping for ");
        Serial.print(SLEEP_MINUTES);
        Serial.println(" minutes");

        // Configure wake-up sources
        touchSleepWakeUpEnable(BUTTON_PIN, TOUCH_THRESHOLD); // Enable touch wake-up on GPIO 18 (TOUCH9)
        esp_sleep_enable_timer_wakeup(SLEEP_MINUTES * 60 * 1000000ULL);

        // Start next 10-minute cycle after reset
        rtc_remaining_sleep_us = SLEEP_MINUTES * 60 * 1000000ULL;
        gettimeofday(&rtc_sleep_enter_time, NULL);

        esp_deep_sleep_start();
    }
    // Timer wake-up - send data to AIO and start new cycle
    Serial.println("Timer wake-up: send data and start new 10-minute cycle");

    // Send data to Adafruit IO (includes accurate measurement)
    sendDataToAIO();

    Serial.print("Sleeping for ");
    Serial.print(SLEEP_MINUTES);
    Serial.println(" minutes");

    // Configure wake-up sources
    touchSleepWakeUpEnable(BUTTON_PIN, TOUCH_THRESHOLD);      // Enable touch wake-up on GPIO 18 (TOUCH9)
    rtc_remaining_sleep_us = SLEEP_MINUTES * 60 * 1000000ULL; // Reset cycle window
    esp_sleep_enable_timer_wakeup(rtc_remaining_sleep_us);
    gettimeofday(&rtc_sleep_enter_time, NULL);

    esp_deep_sleep_start();
}

void loop()
{
    // Not used - everything happens in setup() before deep sleep
}