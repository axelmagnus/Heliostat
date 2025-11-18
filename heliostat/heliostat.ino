#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "config.h"  // Adafruit IO credentials
#include <AdafruitIO_WiFi.h>
#include <driver/gpio.h>
#include <esp_sleep.h>
#include <Adafruit_INA219.h>

// Global variables for blip measurement and ratio
int blipCountMeasured = 0;
float blipRatio = 0.0;
// --- Blip counting for burst current measurement ---
#define BLIP_PIN A2  // Example pin for blip detection
volatile int blipCount = 0;
#define SLEEP_MINUTES 10
float azimuth = 180.0;
float elevation = 45.0;
//#define BATTERY_PIN A13 // Example ADC pin for battery measurement

#define SERVO_FREQ 50  // Hz
int ENPin = 13;        // to shut off the booster

// Location for Malmö, SE
float latitude = 55.6;   // degrees
float longitude = 13.0;  // degrees

float busVoltage;
float current;

// Interrupt Service Routine: only increment the counter
void IRAM_ATTR onBlip() {
  blipCount++;
}

// Attach interrupt, count blips for a fixed window, then detach
// No timing inside ISR; all timing in main code
float measureBlipCurrent(uint8_t pin, uint16_t windowMs) {
  blipCount = 0;
  attachInterrupt(digitalPinToInterrupt(pin), onBlip, FALLING);
  unsigned long start = millis();
  while (millis() - start < windowMs) {
    // Wait for windowMs, ISR counts blips
  }
  detachInterrupt(digitalPinToInterrupt(pin));
  // Map blipCount to current (tuning required)
  // Example: 1 blip = 1 mA (adjust as needed)
  float mappedCurrent = blipCount * 1.0;  // TODO: tune this factor
  Serial.print("Blip count: ");
  Serial.print(blipCount);
  Serial.print(", mapped current: ");
  Serial.println(mappedCurrent);
  // Return blip count  ratio
  return blipCount / windowMs;
}
// Heliostat controller with deep sleep (ESP32)


// --- Tuning constants ---
struct ServoConfig {
  uint8_t azimuthChannel = 0;
  float azMinDeg = 80.0;      // Minimum azimuth in degrees (e.g., east)
  float azMaxDeg = 280.0;     // Maximum azimuth in degrees (e.g., west)
  uint16_t azMinPulse = 480;  // Pulse for azMinDeg
  uint16_t azMaxPulse = 100;  // Pulse for azMaxDeg

  uint8_t elevationChannel = 1;
  float elMinDeg = 30.0;      // Minimum elevation in degrees (from horizon)
  float elMaxDeg = 90.0;      // Maximum elevation in degrees (straight up)
  uint16_t elMinPulse = 480;  // Pulse for elMinDeg
  uint16_t elMaxPulse = 370;  // Pulse for elMaxDeg
};
ServoConfig servoConfig;


Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// --- Adafruit IO Time Subscription ---
AdafruitIO_Time *iso = io.time(AIO_TIME_ISO);
// set up the group
AdafruitIO_Group *group = io.group("heliostat");

Adafruit_INA219 ina219;

volatile time_t latestTime = 0;

// Parse ISO-8601 string to time_t (UTC)
time_t parseISO8601(const char *isoStr) {
  int year, month, day, hour, minute, second;
  if (sscanf(isoStr, "%4d-%2d-%2dT%2d:%2d:%2d", &year, &month, &day, &hour, &minute, &second) == 6) {
    struct tm t;
    t.tm_year = year - 1900;
    t.tm_mon = month - 1;
    t.tm_mday = day;
    t.tm_hour = hour;
    t.tm_min = minute;
    t.tm_sec = second;
    t.tm_isdst = 0;

    return mktime(&t);
  }
  return 0;
}

// ISO time callback
void handleISO(char *data, uint16_t len) {
  latestTime = parseISO8601(data);
  // Serial.print("ISO Feed: ");
  // Serial.println(data);
}

// --- Solar position calculation ---
#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232
#define PI 3.14159265358979323846
#define TWO_PI (2.0 * PI)

long JulianDate(int year, int month, int day) {
  if (month <= 2) {
    year--;
    month += 12;
  }
  int A = year / 100;
  int B = 2 - A + A / 4;
  long JD_whole = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524;
  return JD_whole;
}

void breakTime(time_t t, int &year, int &month, int &day, int &hour, int &minute, int &second) {
  struct tm *tm = gmtime(&t);
  year = tm->tm_year + 1900;
  month = tm->tm_mon + 1;
  day = tm->tm_mday;
  hour = tm->tm_hour;
  minute = tm->tm_min;
  second = tm->tm_sec;
}

void calcSolarAzEl(time_t t, float latitude_deg, float longitude_deg, float &azimuth_deg, float &elevation_deg) {
  float Latitude = latitude_deg * DEG_TO_RAD;
  float Longitude = longitude_deg * DEG_TO_RAD;

  const float DAYS_PER_JULIAN_CENTURY = 36525.0;
  const long Y2K_JULIAN_DAY = 2451545;

  int year, month, day, hour, minute, second;
  breakTime(t, year, month, day, hour, minute, second);

  long JD_whole = JulianDate(year, month, day);
  float JD_frac = (hour + minute / 60.0 + second / 3600.0) / 24.0 - 0.5;

  float elapsedT = JD_whole - Y2K_JULIAN_DAY;
  elapsedT = (elapsedT + JD_frac) / DAYS_PER_JULIAN_CENTURY;

  float solarLongitude = DEG_TO_RAD * fmod(280.46645 + 36000.76983 * elapsedT, 360);
  float solarMeanAnomaly = DEG_TO_RAD * fmod(357.5291 + 35999.0503 * elapsedT, 360);
  float earthOrbitEccentricity = 0.016708617 - 0.000042037 * elapsedT;

  float sunCenter = DEG_TO_RAD * ((1.9146 - 0.004847 * elapsedT) * sin(solarMeanAnomaly) + (0.019993 - 0.000101 * elapsedT) * sin(2 * solarMeanAnomaly) + 0.00029 * sin(3 * solarMeanAnomaly));

  float solarTrueAnomaly = solarMeanAnomaly + sunCenter;
  float equatorObliquity = DEG_TO_RAD * (23 + 26 / 60. + 21.448 / 3600. - 46.815 / 3600 * elapsedT);

  long JDx = JD_whole - Y2K_JULIAN_DAY;
  float GreenwichHourAngle = 280.46061837 + (360 * JDx) % 360 + .98564736629 * JDx + 360.98564736629 * JD_frac;
  GreenwichHourAngle = fmod(GreenwichHourAngle, 360.0);

  float solarTrueLongitude = fmod(sunCenter + solarLongitude, TWO_PI);

  float rightAscension = atan2(sin(solarTrueLongitude) * cos(equatorObliquity), cos(solarTrueLongitude));
  float Declination = asin(sin(equatorObliquity) * sin(solarTrueLongitude));
  float hourAngle = DEG_TO_RAD * GreenwichHourAngle + Longitude - rightAscension;

  azimuth_deg = (PI + atan2(sin(hourAngle), cos(hourAngle) * sin(Latitude) - tan(Declination) * cos(Latitude))) * RAD_TO_DEG;

  elevation_deg = asin(sin(Latitude) * sin(Declination) + cos(Latitude) * cos(Declination) * cos(hourAngle)) * RAD_TO_DEG;
}

// --- Map degrees to servo pulse ---
uint16_t mapAzimuthToPulse(float azDeg) {
  // Constrain to config range
  azDeg = constrain(azDeg, servoConfig.azMinDeg, servoConfig.azMaxDeg);
  return map(azDeg,
             servoConfig.azMinDeg, servoConfig.azMaxDeg,
             servoConfig.azMinPulse, servoConfig.azMaxPulse);
}

uint16_t mapElevationToPulse(float elDeg) {
  elDeg = constrain(elDeg, servoConfig.elMinDeg, servoConfig.elMaxDeg);
  return map(elDeg,
             servoConfig.elMinDeg, servoConfig.elMaxDeg,
             servoConfig.elMinPulse, servoConfig.elMaxPulse);
}
void collectData() {
  Serial.println("Collect data.");
  // Measure data from the INA219 
  // Measure battery with inbuilt ADC
  float batteryLevel = 3.33;  //analogRead(BATTERY_PIN) / 4096.0 * 3.3 * 2 * 1.1;
  Serial.print("Battery Level: ");
  Serial.println(batteryLevel);
  Serial.print("ADC on CHRG:");
  Serial.println(analogRead(BLIP_PIN));

  if (!ina219.begin()) {
    Serial.println("Failed to find INA219 chip");
    busVoltage = batteryLevel;
  } else {  // measure
    delay(200);
    busVoltage = ina219.getBusVoltage_V();
    if (ina219.getBusVoltage_V() < 3.3) {
      Serial.println("Bus voltage low, going to sleep.");
      // Blink LED 3 times
      for (int i = 0; i < 3; i++) {
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
    for (int i = 0; i < 500; i++) {
      current += ina219.getCurrent_mA();
    }
    current /= 500;
    Serial.println(millis());

    Serial.print("INA219 avg current: ");
    Serial.println(current);

    Serial.print("Bus Voltage: ");
    Serial.print(busVoltage);

    Serial.println(" V");
  }
/*
  // Blip-based current measurement (100 ms window)
  blipCountMeasured = measureBlipCurrent(BLIP_PIN, 100);
  blipRatio = 0.0;
  // Serial.print("Blip count: ");
  // Serial.println(blipCountMeasured);
  if (blipCountMeasured > 0) {
    blipRatio = current / blipCountMeasured;
    Serial.print("INA219 current / blip count ratio: ");
    Serial.println(blipRatio, 4);
  } else {
    Serial.println("No blips detected, ratio undefined.");
  }
  */
}

void send_data(float azi, float elv) {
  // Send data to Adafruit IO using group publish
  Serial.println("Sending data");
  //int chrgadc = analogRead(BLIP_PIN);
  group->set("bus_voltage", busVoltage);
  group->set("current", current);
  group->set("elevation", elv);
  group->set("azimuth", azi);
  //group->set("blip_count", blipCountMeasured);
  //group->set("blip_ratio", blipRatio);
  //group->set("chrg_adc", chrgadc);
  group->save();
  io.run();
}

void setup() {
  Serial.begin(115200);
  //Serial.println("Collecting data..");
  //collectData();
  pinMode(ENPin, OUTPUT);
  digitalWrite(ENPin, LOW); //turn off booster
  pinMode(13, OUTPUT);
  //pinMode(BLIP_PIN, INPUT);
  //pinMode(BATTERY_PIN, INPUT);

  // Adafruit IO setup
  io.connect();
  iso->onMessage(handleISO);
  Serial.println(WIFI_SSID);
  int tries = 0;

  while (io.status() < AIO_CONNECTED && tries < 20) {
    Serial.print(tries++);
    digitalWrite(13, tries % 2 + 1);
    Serial.println(io.statusText());
    delay(500);
  }

  Serial.println();
  Serial.println(io.statusText());
  delay(10);

  // Wait for ISO time to arrive
  unsigned long start = millis();
  while (latestTime < 1000000000 && millis() - start < 10000) {  // wait max 10s
    io.run();
    Serial.println("io.run");
    delay(100);
  }

  if (latestTime > 1000000000)  // Got internet and time, get direction
  {
    calcSolarAzEl(latestTime, latitude, longitude, azimuth, elevation);
    
    uint16_t azPulse = mapAzimuthToPulse(azimuth);
    uint16_t elPulse = mapElevationToPulse(elevation);

    // Only cahnge the panel if elevation is above 0, that is, the sun is up
    if (elevation > 0) {
      Serial.println("Directing panel..");
      // Enable power boost
      digitalWrite(ENPin, HIGH);
      delay(500);
      // start the servos if you got time

      pwm.begin();
      pwm.setOscillatorFrequency(27000000);
      pwm.setPWMFreq(SERVO_FREQ);

      pwm.setPWM(servoConfig.azimuthChannel, 0, azPulse);
      delay(1000);  //One at a time to limit current draw
      // Turn off azimuth and elevation servos
      pwm.setPWM(servoConfig.azimuthChannel, 0, 0);
      delay(400);

      pwm.setPWM(servoConfig.elevationChannel, 0, elPulse);
      delay(1000);
      pwm.setPWM(servoConfig.elevationChannel, 0, 0);
      delay(300);
      // Turn off boost before measuring
      digitalWrite(ENPin, LOW);
      delay(400);

      Serial.print("Azimuth: ");
      Serial.print(azimuth);
      Serial.print(" deg, pulse: ");
      Serial.println(azPulse);
      Serial.print("Elevation: ");
      Serial.print(elevation);
      Serial.print(" deg, pulse: ");
      Serial.println(elPulse);
    }
    collectData();
    send_data(azimuth, elevation);  //send before trying to move servos, TODO: differenet first mesurement in the dawn
  } else {
    Serial.println("No valid time received.");
  }
  //pull down ENpin to keep it off in deep sleep
  gpio_pulldown_en((gpio_num_t)ENPin);
  gpio_pullup_dis((gpio_num_t)ENPin);
  Serial.println("Sleeping for 10 minutes...");
  esp_deep_sleep(SLEEP_MINUTES * 60 * 1000000ULL);  // 10 minutes in microseconds
}

void loop() {
  // Not used; everything is in setup()
}