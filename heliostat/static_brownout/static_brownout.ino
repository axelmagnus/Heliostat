// Static solar panel monitor with brownout recovery - no voltage check
// Uses brownout reset detection to trigger extended sleep for battery recovery
// Sends data to Adafruit IO without display to minimize power consumption

#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_SHT4x.h>
#include <AdafruitIO_WiFi.h>
#include "config.h"  // Adafruit IO credentials
#include <esp_sleep.h>
#include <esp_system.h>

#define SLEEP_MINUTES 10
#define BROWNOUT_RECOVERY_MINUTES 30
#define ADC_SAMPLE_SIZE 500

Adafruit_INA219 ina219;
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
AdafruitIO_Group *group = io.group("heliostat");

// RTC memory to persist brownout count across resets
RTC_DATA_ATTR int rtc_brownout_count = 0;
RTC_DATA_ATTR uint64_t rtc_last_brownout_time = 0;

float busVoltage = 0;
float current = 0;
float temperature = 0;
float humidity = 0;

void collectData() {
  Serial.println("Collecting sensor data...");

  // Measure INA219 voltage and current
  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    busVoltage = 0;
    current = 0;
  } else {
    busVoltage = ina219.getBusVoltage_V();
    
    // Accurate 500-sample current measurement
    current = 0;
    unsigned long start = millis();
    for (int i = 0; i < ADC_SAMPLE_SIZE; i++) {
      current += ina219.getCurrent_mA();
    }
    current /= ADC_SAMPLE_SIZE;
    
    Serial.print("Voltage: ");
    Serial.print(busVoltage);
    Serial.print(" V, Current: ");
    Serial.print(current);
    Serial.println(" mA");
  }

  // Measure SHT41 temperature and humidity
  if (!sht4.begin()) {
    Serial.println("Failed to find SHT41 chip");
    temperature = 0;
    humidity = 0;
  } else {
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
}

void sendDataToAIO() {
  Serial.println("Connecting to Adafruit IO...");
  Serial.println(WIFI_SSID);
  io.connect();

  int aioTries = 0;
  while (io.status() < AIO_CONNECTED && aioTries < 20) {
    Serial.println(io.statusText());
    delay(500);
    aioTries++;
  }

  if (io.status() < AIO_CONNECTED) {
    Serial.println("Failed to connect to AIO, skipping data send");
    return;
  }

  Serial.println("Connected to AIO");

  // Publish to Adafruit IO with stat_ prefix
  Serial.print("Publishing stat_bus_voltage: ");
  Serial.println(busVoltage);
  Serial.print("Publishing stat_current: ");
  Serial.println(current);
  Serial.print("Publishing stat_temperature: ");
  Serial.println(temperature);
  Serial.print("Publishing stat_humidity: ");
  Serial.println(humidity);
  Serial.print("Publishing stat_brownout_count: ");
  Serial.println(rtc_brownout_count);

  group->set("stat_bus_voltage", busVoltage);
  group->set("stat_current", current);
  group->set("stat_temperature", temperature);
  group->set("stat_humidity", humidity);
  group->set("stat_brownout_count", rtc_brownout_count);
  group->save();

  unsigned long startTime = millis();
  unsigned long runDuration = 5000;  // Run io.run() for 5 seconds

  while (millis() - startTime < runDuration) {
    io.run();
    if (io.status() == AIO_CONNECTED) {
      Serial.println("Data successfully sent to Adafruit IO");
      break;
    }
    delay(10);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Check reset reason
  esp_reset_reason_t reset_reason = esp_reset_reason();
  
  Serial.print("Reset reason: ");
  switch (reset_reason) {
    case ESP_RST_UNKNOWN:
      Serial.println("UNKNOWN");
      break;
    case ESP_RST_POWERON:
      Serial.println("POWERON");
      rtc_brownout_count = 0;  // Reset counter on power-on
      break;
    case ESP_RST_EXT:
      Serial.println("EXTERNAL PIN");
      break;
    case ESP_RST_SW:
      Serial.println("SOFTWARE");
      break;
    case ESP_RST_PANIC:
      Serial.println("PANIC");
      break;
    case ESP_RST_INT_WDT:
      Serial.println("INTERRUPT WATCHDOG");
      break;
    case ESP_RST_TASK_WDT:
      Serial.println("TASK WATCHDOG");
      break;
    case ESP_RST_WDT:
      Serial.println("WATCHDOG");
      break;
    case ESP_RST_DEEPSLEEP:
      Serial.println("DEEP SLEEP");
      break;
    case ESP_RST_BROWNOUT:
      Serial.println("BROWNOUT - Battery too low during WiFi/operation!");
      rtc_brownout_count++;
      rtc_last_brownout_time = esp_timer_get_time();
      break;
    case ESP_RST_SDIO:
      Serial.println("SDIO");
      break;
    default:
      Serial.println("OTHER");
      break;
  }

  // If brownout detected, sleep for extended recovery period
  if (reset_reason == ESP_RST_BROWNOUT) {
    Serial.print("Brownout detected (count: ");
    Serial.print(rtc_brownout_count);
    Serial.print("). Sleeping for ");
    Serial.print(BROWNOUT_RECOVERY_MINUTES);
    Serial.println(" minutes to allow battery recovery...");
    
    // Blink to indicate brownout recovery mode (if there's an LED)
    pinMode(13, OUTPUT);
    for (int i = 0; i < 5; i++) {
      digitalWrite(13, HIGH);
      delay(200);
      digitalWrite(13, LOW);
      delay(200);
    }
    
    esp_sleep_enable_timer_wakeup(BROWNOUT_RECOVERY_MINUTES * 60 * 1000000ULL);
    esp_deep_sleep_start();
  }

  // Normal operation - collect and send data
  Serial.println("Normal operation: collecting and sending data");
  
  collectData();
  sendDataToAIO();

  Serial.print("Sleeping for ");
  Serial.print(SLEEP_MINUTES);
  Serial.println(" minutes");

  esp_sleep_enable_timer_wakeup(SLEEP_MINUTES * 60 * 1000000ULL);
  esp_deep_sleep_start();
}

void loop() {
  // Not used - everything happens in setup() before deep sleep
}
