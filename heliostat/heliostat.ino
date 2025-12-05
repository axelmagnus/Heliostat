#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "config.h" // Adafruit IO credentials
#include <AdafruitIO_WiFi.h>
#include <driver/gpio.h>  // Required for low-level GPIO control
#include <esp_sleep.h>    // Required for deep sleep functions
#include <esp_task_wdt.h> // Required for watchdog timer control
#include <Adafruit_INA219.h>
// Added peripherals for external wake displays
#include <Adafruit_SHT4x.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "Adafruit_MAX1704X.h"
// Use Arduino Time library for time calculations
#include <TimeLib.h>

// Ensure TFT backlight pin is defined (guard against missing macro)
#ifndef TFT_BACKLIGHT
#define TFT_BACKLIGHT 45
#endif

// I2C power control pin for ESP32-S3 Reverse TFT
#ifndef TFT_I2C_POWER
#define TFT_I2C_POWER 21
#endif

#define SLEEP_MINUTES 10
float azimuth = 180.0;
float elevation = 45.0;
// #define BATTERY_PIN A13 // Example ADC pin for battery measurement

#define SERVO_FREQ 50 // Hz
int ENPin = 12;       // to shut off the booster (Connected to GPIO 12)

// Buttons on ESP32-S3 Reverse TFT (adjust if needed)
#ifndef BUTTON_D1
#define BUTTON_D1 1
#endif
#ifndef BUTTON_D2
#define BUTTON_D2 2
#endif

// Teal color scheme (same as static_panel)
#define COLOR_DARK_TEAL 0x0410
#define COLOR_MID_TEAL 0x4E9C
#define COLOR_LIGHT_TEAL 0xEF3D
// Auto-calc timezone offset (Europe/Stockholm): +60 winter, +120 DST
int tzOffsetMinutesForUnix(time_t t)
{
  // Compute DST for EU: starts last Sunday of March at 01:00 UTC,
  // ends last Sunday of October at 01:00 UTC.
  struct tm tmUtc = *gmtime(&t);
  int year = tmUtc.tm_year + 1900;

  auto lastSunday = [](int y, int month)
  {
    // Find last Sunday of given month (1-12)
    struct tm tm = {};
    tm.tm_year = y - 1900;
    tm.tm_mon = month - 1;
    tm.tm_mday = 31; // max; mktime will normalize
    tm.tm_hour = 1;  // 01:00 UTC boundary

    // Convert UTC tm to time_t using TimeLib
    tmElements_t te;
    te.Year = (y - 1970); // TimeLib Year is years since 1970
    te.Month = month;
    te.Day = tm.tm_mday; // preserved 31 normalized by TimeLib
    te.Hour = tm.tm_hour;
    te.Minute = 0; // boundary hour already set
    te.Second = 0;
    time_t tt = makeTime(te);
    tm = *gmtime(&tt);
    // Walk back to Sunday
    int backDays = (tm.tm_wday + 7 - 0) % 7; // 0=Sunday
    return tt - backDays * 24 * 3600;
  };

  time_t dstStart = lastSunday(year, 3); // March
  time_t dstEnd = lastSunday(year, 10);  // October

  // Between start and end → DST (+120), else standard (+60)
  return (t >= dstStart && t < dstEnd) ? 120 : 60;
}

// Removed custom timegm_utc in favor of TimeLib's makeTime
// Location for Malmö, SE
float latitude = 55.6;  // degrees
float longitude = 13.0; // degrees

float busVoltage;
float current;
float currentWifi; // measured after servo movment
float cellVoltage; // from MAX17048
float chargeRate;  // from MAX17048

// RTC persisted quick/accurate readings for fast EXT1 wake display
RTC_DATA_ATTR float rtc_busVoltage = 3.33f;
RTC_DATA_ATTR float rtc_current = 0.0f; // Accurate 500-sample current
RTC_DATA_ATTR float rtc_temperature = 0.0f;
RTC_DATA_ATTR float rtc_humidity = 0.0f;
RTC_DATA_ATTR float rtc_battPercent = 4.0f;
RTC_DATA_ATTR float rtc_cellVoltage = 3.7f;
RTC_DATA_ATTR float rtc_chargeRate = 0.0f;
RTC_DATA_ATTR int rtc_remaining_sleep_sec = 0;              // remaining until next timer wake in seconds
RTC_DATA_ATTR struct timeval rtc_sleep_enter_time = {0, 0}; // time when deep sleep entered
RTC_DATA_ATTR time_t rtc_last_unix = 0;                     // last known Unix time (UTC) from Adafruit IO
RTC_DATA_ATTR float rtc_elevation = 40.0f;                  // last calculated solar elevation
RTC_DATA_ATTR float rtc_azimuth = 177.5f;                   // last calculated solar azimuth

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// --- Adafruit IO Time Subscription ---
// Assuming 'io' is defined in config.h
AdafruitIO_Time *iso = io.time(AIO_TIME_ISO);
// set up the group
AdafruitIO_Group *group = io.group("heliostat");

Adafruit_INA219 ina219;
Adafruit_SHT4x sht4;
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_MAX17048 maxlipo;

// Forward declarations
void readSensorsQuick();
void readAccurateCurrent();
void renderEnv(float temperature, float humidity, float elevDeg, time_t unixTime);
void renderBattery(float percent, float cellV, float busV, float currmA);
void renderStatus(time_t unixTime, float elevDeg, float azDeg);
void calcRemainingTime();

// Fast render of stored SHT41 values (no sensor I/O)
void renderEnv(float temperature, float humidity, float elevDeg, time_t unixTime)
{
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);
  tft.init(135, 240);
  tft.setRotation(1);
  tft.fillScreen(COLOR_DARK_TEAL);
  // Same 2x2 grid as battery screen
  tft.fillRoundRect(5, 5, 110, 55, 8, COLOR_MID_TEAL);    // UL
  tft.fillRoundRect(125, 5, 110, 55, 8, COLOR_MID_TEAL);  // UR
  tft.fillRoundRect(5, 70, 110, 55, 8, COLOR_MID_TEAL);   // LL
  tft.fillRoundRect(125, 70, 110, 55, 8, COLOR_MID_TEAL); // LR

  tft.setTextColor(COLOR_DARK_TEAL);
  // Upper-left: Temp
  tft.setTextSize(2);
  tft.setCursor(15, 15);
  tft.print("Temp");
  tft.setTextSize(3);
  tft.setCursor(15, 35);
  tft.print(isnan(temperature) ? 0 : temperature, 1);
  tft.setTextSize(2);
  tft.print(" C");

  // Upper-right: Elevation
  tft.setTextSize(2);
  tft.setCursor(135, 15);
  tft.print("Elev");
  tft.setTextSize(3);
  tft.setCursor(135, 35);
  tft.print(elevDeg, 0);

  tft.setTextSize(4);
  tft.print(" \xB0");

  // Change degree symbol from CP437 0xF8 to 0xB0

  // Lower-left: Humidity
  tft.setTextSize(2);
  tft.setCursor(15, 80);
  tft.print("Hum");
  tft.setTextSize(3);
  tft.setCursor(15, 100);
  tft.print(isnan(humidity) ? 0 : humidity, 0);
  tft.setTextSize(2);
  tft.print(" %");

  // Lower-right: Time HH:MM (auto timezone offset EU)
  time_t localTime = unixTime + (time_t)(tzOffsetMinutesForUnix(unixTime) * 60);
  struct tm *tmv = gmtime(&localTime);
  if (tmv)
  {
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", tmv->tm_hour, tmv->tm_min);
    tft.setTextSize(3);
    tft.setCursor(135, 80);
    tft.print(buf);
    tft.setTextSize(2);
    tft.setCursor(135, 105);
    tft.print(WIFI_SSID);
  }
}

// Status splash screen with time, elevation, azimuth, and SSID
void renderStatus(time_t unixTime, float elevDeg, float azDeg)
{

  tft.fillScreen(COLOR_DARK_TEAL);
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);
  tft.init(135, 240);
  tft.setRotation(1);
  tft.setTextColor(COLOR_LIGHT_TEAL);

  // Time at top (auto timezone offset EU)
  time_t localTime = unixTime + (time_t)(tzOffsetMinutesForUnix(unixTime) * 60);
  struct tm *tmv = gmtime(&localTime);
  if (tmv)
  {
    char buf[6];
    snprintf(buf, sizeof(buf), "%02d:%02d", tmv->tm_hour, tmv->tm_min);
    tft.setTextSize(4);
    tft.setCursor(80, 10);
    tft.print(buf);
  }

  // Elevation and Azimuth on next row
  tft.setTextSize(2);
  tft.setCursor(10, 50);
  tft.print("Elev: ");
  tft.print(elevDeg, 1);
  tft.print(" \xB0");

  // Change degree symbol from CP437 0xF8 to 0xB0

  tft.setCursor(10, 70);
  tft.print("Azim: ");
  tft.print(azDeg, 1);
  tft.print(" \xB0");

  // Change degree symbol from CP437 0xF8 to 0xB0

  // Connected SSID at bottom
  tft.setTextSize(2);
  tft.setCursor(10, 100);
  tft.print("Connected.");

  tft.setCursor(10, 120);
  tft.print("Sending data..");
}

// Fast render of stored battery / power values
void renderBattery(float percent, float cellV, float currmA)
{
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);
  tft.init(135, 240);
  tft.setRotation(1);
  tft.fillScreen(COLOR_DARK_TEAL);
  tft.fillRoundRect(5, 5, 110, 55, 8, COLOR_MID_TEAL);
  tft.fillRoundRect(125, 5, 110, 55, 8, COLOR_MID_TEAL);
  tft.fillRoundRect(5, 70, 110, 55, 8, COLOR_MID_TEAL);
  tft.fillRoundRect(125, 70, 110, 55, 8, COLOR_MID_TEAL);
  tft.setTextColor(COLOR_DARK_TEAL);
  tft.setTextSize(2);
  tft.setCursor(15, 15);
  tft.print("Batt. V");
  tft.setTextSize(3);
  tft.setCursor(15, 35);
  tft.print(cellV, 2);
  tft.setTextSize(2);
  tft.setCursor(135, 15);
  tft.print("Batt %");
  tft.setTextSize(3);
  tft.setCursor(135, 35);
  tft.print(isnan(percent) ? 0 : percent, 1);
  tft.setTextSize(2);
  tft.setCursor(15, 80);
  tft.print("Curr. mA");
  tft.setTextSize(3);
  tft.setCursor(15, 100);
  tft.print(currmA, 1);
  tft.setTextSize(2);
  tft.setCursor(135, 80);
  tft.print("Sleep");
  tft.setTextSize(3);
  tft.setCursor(135, 100);
  tft.print(rtc_remaining_sleep_sec);
  // tft.setTextSize(2);
  tft.print(" s");
}

// --- Tuning constants ---
struct ServoConfig
{
  uint8_t azimuthChannel = 4;
  float azMinDeg = 80.0;     // Azimuth 80° (pulse 495)
  float azMaxDeg = 275.0;    // Azimuth 275° (pulse 110)
  uint16_t azMinPulse = 495; // Pulse for azMinDeg (80°)
  uint16_t azMaxPulse = 110; // Pulse for azMaxDeg (275°)

  uint8_t elevationChannel = 5;
  float elMinDeg = 0.0;      // Elevation 0° (pulse 490)
  float elMaxDeg = 80.0;     // Elevation 80° (pulse 350)
  uint16_t elMinPulse = 490; // Pulse for elMinDeg (0°)
  uint16_t elMaxPulse = 350; // Pulse for elMaxDeg (80°)
};
ServoConfig servoConfig;
constexpr uint8_t PCA9685_MODE1 = 0x00;
constexpr uint8_t PCA9685_MODE1_SLEEP = 0x10;
constexpr uint8_t PCA9685_MODE1_AI = 0x20;
volatile time_t latestTime = 0;

// Parse ISO-8601 string to time_t (UTC)
time_t parseISO8601(const char *isoStr)
{
  int year, month, day, hour, minute, second;
  // Use T%2d to match the 'T' separator and the following hour digits
  if (sscanf(isoStr, "%4d-%2d-%2dT%2d:%2d:%2d", &year, &month, &day, &hour, &minute, &second) == 6)
  {
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
void handleISO(char *data, uint16_t len)
{
  latestTime = parseISO8601(data);
  if (latestTime > 0)
  {
    rtc_last_unix = latestTime;
  }
  // Serial.print("ISO Feed: ");
  // Serial.println(data);
}

// --- Solar position calculation ---
#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232
#define PI 3.14159265358979323846
#define TWO_PI (2.0 * PI)

long JulianDate(int year, int month, int day)
{
  if (month <= 2)
  {
    year--;
    month += 12;
  }
  int A = year / 100;
  int B = 2 - A + A / 4;
  long JD_whole = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524;
  return JD_whole;
}

void breakTime(time_t t, int &year, int &month, int &day, int &hour, int &minute, int &second)
{
  struct tm *tm = gmtime(&t);
  year = tm->tm_year + 1900;
  month = tm->tm_mon + 1;
  day = tm->tm_mday;
  hour = tm->tm_hour;
  minute = tm->tm_min;
  second = tm->tm_sec;
}

void calcSolarAzEl(time_t t, float latitude_deg, float longitude_deg, float &azimuth_deg, float &elevation_deg)
{
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
uint16_t mapAzimuthToPulse(float azDeg)
{
  // Constrain to config range
  azDeg = constrain(azDeg, servoConfig.azMinDeg, servoConfig.azMaxDeg);
  return map(azDeg,
             servoConfig.azMinDeg, servoConfig.azMaxDeg,
             servoConfig.azMinPulse, servoConfig.azMaxPulse);
}

uint16_t mapElevationToPulse(float elDeg)
{
  elDeg = constrain(elDeg, servoConfig.elMinDeg, servoConfig.elMaxDeg);
  return map(elDeg,
             servoConfig.elMinDeg, servoConfig.elMaxDeg,
             servoConfig.elMinPulse, servoConfig.elMaxPulse);
}

// Quick one-shot reads for temp/humidity + battery percent
void readSensorsQuick()
{
  // Enable I2C power and initialize sensors (only happens on timer/reset wake)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
  delay(250); // MAX17048 needs time to stabilize after power-up

  Serial.println("I2C power enabled");

  maxlipo.begin();
  sht4.begin();
  ina219.begin();

  delay(100); // Additional settling time after begin() calls

  // SHT41
  Serial.println("Reading SHT41...");
  sensors_event_t he, te;
  sht4.getEvent(&he, &te);
  if (!isnan(te.temperature))
  {
    rtc_temperature = te.temperature;
    rtc_humidity = he.relative_humidity;
    Serial.println("SHT41 read OK");
  }
  else
  {
    Serial.println("SHT41 read invalid, keeping cached values");
  }

  // Fuel gauge - wait for device ready with timeout
  Serial.println("Reading MAX17048...");
  int maxRetries = 10;
  bool maxReady = false;
  for (int i = 0; i < maxRetries; i++)
  {
    if (maxlipo.isDeviceReady())
    {
      maxReady = true;
      break;
    }
    delay(50);
  }

  if (maxReady)
  {
    rtc_cellVoltage = maxlipo.cellVoltage();
    rtc_battPercent = maxlipo.cellPercent();
    rtc_chargeRate = maxlipo.chargeRate();
    Serial.println("MAX17048 ready and read OK");
  }
  else
  {
    Serial.print("MAX17048 not ready after ");
    Serial.print(maxRetries * 50);
    Serial.println(" ms, keeping cached values");
  }

  Serial.print("MAX17048 status: voltage=");
  Serial.print(rtc_cellVoltage);
  Serial.print(" V, SOC=");
  Serial.print(rtc_battPercent);
  Serial.print(" %");
  Serial.print(" Charge Rate=");
  Serial.print(rtc_chargeRate);
  Serial.println(" %/h");

  Serial.print("Temp: ");
  Serial.print(rtc_temperature);
  Serial.print(" C, Humidity: ");
  Serial.print(rtc_humidity);
  Serial.println(" %");
  Serial.print("Battery: ");
  Serial.print(rtc_battPercent);
  Serial.print(" %, Cell V: ");
  Serial.print(rtc_cellVoltage);
  Serial.println(" V");
}

// Calculate remaining sleep time based on elapsed time since sleep entry
void calcRemainingTime()
{
  struct timeval now;
  gettimeofday(&now, NULL);
  int elapsed_sec = 0;

  if (rtc_sleep_enter_time.tv_sec != 0)
  {
    elapsed_sec = (int)(now.tv_sec - rtc_sleep_enter_time.tv_sec);
  }

  if (elapsed_sec < rtc_remaining_sleep_sec)
  {
    rtc_remaining_sleep_sec -= elapsed_sec;
  }
  else
  {
    rtc_remaining_sleep_sec = 0;
  }

  Serial.print("Remaining sleep: ");
  Serial.print(rtc_remaining_sleep_sec);
  Serial.println(" seconds");
}

// Accurate current averaging (500 samples) only for reset wake
void readAccurateCurrent()
{
  if (!ina219.begin())
    return;
  float acc = 0;
  for (int i = 0; i < 500; i++)
  {
    acc += ina219.getCurrent_mA();
  }
  rtc_current = acc / 500.0f;
  Serial.print("Accurate 500-sample current: ");
  Serial.println(rtc_current);
}

// Prepare pins, wake sources, compute remaining sleep, and enter deep sleep
void go2sleep(int seconds)
{
  // 1. Ensure ENPin LOW and held during sleep
  digitalWrite(ENPin, LOW);

  digitalWrite(TFT_I2C_POWER, LOW);
  gpio_set_level((gpio_num_t)ENPin, 0);
  gpio_pulldown_en((gpio_num_t)ENPin);
  gpio_pullup_dis((gpio_num_t)ENPin);
  gpio_hold_en((gpio_num_t)ENPin);

  // 2. Configure button pins with pulldowns and hold state during sleep
  gpio_pulldown_en((gpio_num_t)BUTTON_D1);
  gpio_pullup_dis((gpio_num_t)BUTTON_D1);
  gpio_hold_en((gpio_num_t)BUTTON_D1);

  gpio_pulldown_en((gpio_num_t)BUTTON_D2);
  gpio_pullup_dis((gpio_num_t)BUTTON_D2);
  gpio_hold_en((gpio_num_t)BUTTON_D2);

  gpio_deep_sleep_hold_en();

  // Use the passed seconds parameter for sleep duration
  uint64_t sleep_us = (uint64_t)seconds * 1000000ULL;
  Serial.print("Sleeping for ");
  Serial.print(seconds);
  Serial.println(" seconds");

  // External button wake
  uint64_t ext1_mask = (1ULL << BUTTON_D1) | (1ULL << BUTTON_D2);
  esp_sleep_enable_ext1_wakeup(ext1_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_sleep_enable_timer_wakeup(sleep_us);
  gettimeofday(&rtc_sleep_enter_time, NULL);
  Serial.flush();
  delay(50);
  esp_deep_sleep_start();
}

void send_data(float azi, float elv)
{
  Serial.println("Sending data to Adafruit IO...");

  // Disable watchdog during publish
  esp_task_wdt_delete(xTaskGetIdleTaskHandleForCPU(0));

  // Send in smaller batches with generous delays
  // Batch 1: Position data
  group->set("elevation", elv);
  group->set("azimuth", azi);
  group->set("current", rtc_current);
  group->save();

  for (int i = 0; i < 10; i++)
  {
    io.run(100);
    delay(100);
  }

  Serial.println("Position data sent");

  // Clear group to prevent accumulation
  group = io.group("heliostat");

  // Batch 2: Environmental data
  group->set("temperature", rtc_temperature);
  group->set("humidity", rtc_humidity);
  group->save();

  for (int i = 0; i < 10; i++)
  {
    io.run(100);
    delay(100);
  }

  Serial.println("Environmental data sent");

  // Clear group to prevent accumulation
  group = io.group("heliostat");

  // Batch 3: Battery data - combine to reduce publishes
  group->set("battery_percent", rtc_battPercent);
  group->set("cell_voltage", rtc_cellVoltage);
  group->save();
  for (int i = 0; i < 10; i++)
  {
    io.run(100);
    delay(100);
  }
  Serial.println("Battery data sent");

  // Skip charge_rate for now - may be causing issues
  // group->set("charge_rate", rtc_chargeRate);
  // group->save();

  Serial.println("Publish complete.");
}

// --- CRITICAL EARLY PIN STABILIZATION (Fixes GPIO 13 startup glitch) ---
// This function runs automatically BEFORE setup() using the __attribute__((constructor))
// to ensure ENPin (GPIO 12) is LOW immediately on power-up, reset, or wake.
void earlyPinStabilization() __attribute__((constructor));
void earlyPinStabilization()
{
  // 1. Immediately configure the pin to OUTPUT and set level LOW.
  gpio_set_direction((gpio_num_t)ENPin, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)ENPin, 0); // 0 = LOW

  // 2. AGGRESSIVELY apply the hardware hold state now.
  // This locks the pin LOW right after setting the level, preventing external circuits (like the LED pull-up)
  // from briefly pulling it HIGH during the remainder of the boot process.
  gpio_hold_en((gpio_num_t)ENPin);
}
// --- END EARLY STABILIZATION ---

void setup()
{
  Serial.begin(115200);

  Serial.println("\n\n========================================");
  Serial.println("HELIOSTAT BOOT - DEBUG BUILD");
  Serial.println("========================================");

  // Log wake cause immediately
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.print("Wake cause: ");
  switch (cause)
  {
  case ESP_SLEEP_WAKEUP_UNDEFINED:
    Serial.println("RESET/POWER_ON");
    break;
  case ESP_SLEEP_WAKEUP_EXT0:
    Serial.println("EXT0");
    break;
  case ESP_SLEEP_WAKEUP_EXT1:
    Serial.println("EXT1 (button)");
    digitalWrite(TFT_BACKLIGHT, HIGH);
    break;
  case ESP_SLEEP_WAKEUP_TIMER:
    Serial.println("TIMER");
    break;
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    Serial.println("TOUCHPAD");
    break;
  case ESP_SLEEP_WAKEUP_ULP:
    Serial.println("ULP");
    break;
  default:
    Serial.println("OTHER");
    break;
  }

  // --- CRITICAL WAKE-UP STABILIZATION IN SETUP ---
  // If the device woke from deep sleep, the ENPin is currently held LOW by the RTC core.
  // This step is also REQUIRED after a cold boot/reset to release the hold applied
  // in the aggressive earlyPinStabilization constructor.

  // 1. Release the physical hold applied by the aggressive constructor (or previous deep sleep).
  gpio_deep_sleep_hold_dis();
  gpio_hold_dis((gpio_num_t)ENPin);
  gpio_hold_dis((gpio_num_t)BUTTON_D1);
  gpio_hold_dis((gpio_num_t)BUTTON_D2);

  // 2. Re-assert the Arduino high-level configuration.
  pinMode(ENPin, OUTPUT);
  digitalWrite(ENPin, LOW); // Ensure booster is OFF (Servo Enable LOW)

  pinMode(13, OUTPUT);
  // Defer data collection until after EXT1 check so EXT1 path is instant

  // Prepare external wake buttons as active-HIGH (idle LOW)
  // Use pulldowns so ext1 ANY_HIGH triggers on press
  pinMode(BUTTON_D1, INPUT_PULLDOWN);
  pinMode(BUTTON_D2, INPUT_PULLDOWN);

  // If woke by external buttons, show appropriate screen and return to sleep
  if (cause == ESP_SLEEP_WAKEUP_EXT1)
  {
    calcRemainingTime();

    uint64_t status = esp_sleep_get_ext1_wakeup_status();
    bool d1_triggered = status & (1ULL << BUTTON_D1);
    bool d2_triggered = status & (1ULL << BUTTON_D2);

    // Compute current unix time based on elapsed sleep without WiFi
    struct timeval now;
    gettimeofday(&now, NULL);
    int elapsed_sec = 0;
    if (rtc_sleep_enter_time.tv_sec != 0)
    {
      elapsed_sec = (int)(now.tv_sec - rtc_sleep_enter_time.tv_sec);
    }
    time_t current_unix = rtc_last_unix + elapsed_sec;

    if (d2_triggered)
    {
      Serial.println("Button D2 -> Env cached screen");
      renderEnv(rtc_temperature, rtc_humidity, rtc_elevation, current_unix);
    }
    else if (d1_triggered)
    {
      Serial.println("Button D1 -> Battery cached screen");
      renderBattery(rtc_battPercent, rtc_cellVoltage, rtc_current);
    }
    else
    {
      renderEnv(rtc_temperature, rtc_humidity, rtc_elevation, current_unix);
    }

    // Keep screen on briefly, then sleep without running servos/AIO
    delay(4000);
    digitalWrite(TFT_BACKLIGHT, LOW);

    calcRemainingTime();
    Serial.println("Sleeping after external wake display...");
    go2sleep(rtc_remaining_sleep_sec); // Use remaining time in seconds
    return;
  }

  // RESET or timer wake path continues here
  readSensorsQuick(); // SHT41 + fuel gauge cached
  // This shoudl be a reset wake, we may perform accurate current sampling
  readAccurateCurrent();

  // Set full sleep cycle for timer/reset wakes
  rtc_remaining_sleep_sec = SLEEP_MINUTES * 60;

  // Only show screens on reset/power-on, not timer wake
  if (cause == ESP_SLEEP_WAKEUP_UNDEFINED)
  {
    renderBattery(rtc_battPercent, rtc_cellVoltage, rtc_current);
    delay(2000); // Show battery screen for 2s
    // Show environment with elevation and time
    renderEnv(rtc_temperature, rtc_humidity, elevation, rtc_last_unix);
    delay(2000); // Show SHT41 screen for 2s
    // Fill screen and display connecting message
    tft.fillScreen(COLOR_DARK_TEAL);
    tft.setTextColor(COLOR_LIGHT_TEAL);
    tft.setTextSize(2);
    tft.setCursor(10, 50);
    tft.print("Connecting to:");
    tft.setCursor(10, 80);
    tft.print(WIFI_SSID);
  }

  // If voltage is low (<3.3V), skip WiFi and go back to sleep
  if (rtc_cellVoltage < 3.3f)
  {
    Serial.print("Cell voltage low: ");
    Serial.print(rtc_cellVoltage);
    Serial.println("Voltage below 3.3V, skipping WiFi and sleeping.");
    go2sleep(SLEEP_MINUTES * 60);
    return;
  }

  // Adafruit IO setup
  io.connect();
  iso->onMessage(handleISO);
  Serial.println(WIFI_SSID);
  int tries = 0;

  while (io.status() < AIO_CONNECTED && tries < 20)
  {
    Serial.print(tries++);
    digitalWrite(13, tries % 2 == 0 ? HIGH : LOW); // Use ENPin/LED for status blink
    Serial.println(io.statusText());
    delay(500);
  }
  // if wifi connection was not successful, go to sleep
  if (io.status() < AIO_CONNECTED)
  {
    Serial.println("Failed to connect to Adafruit IO, going to sleep.");
    go2sleep(SLEEP_MINUTES * 60);
    return;
  }
  // Turn off LED after connecting
  digitalWrite(13, LOW);
  Serial.println();
  Serial.println(io.statusText());
  delay(10);

  // Wait for ISO time to arrive
  unsigned long start = millis();
  // wait max 10s or until we get a valid time (time_t is non-zero)
  while (latestTime == 0 && millis() - start < 10000)
  {
    io.run();
    Serial.println(io.statusText());
    delay(100);
  }

  if (latestTime > 1000000000) // Got internet and time, get direction, start the servos
  {
    calcSolarAzEl(latestTime, latitude, longitude, azimuth, elevation);
    rtc_elevation = elevation; // Store for button wake displays
    rtc_azimuth = azimuth;

    // Show status splash screen on reset wake after WiFi connection
    if (cause == ESP_SLEEP_WAKEUP_UNDEFINED)
    {
      renderStatus(latestTime, elevation, azimuth);
    }

    

    // Only move servos if sun is up)
    if (elevation > 0) // && cause == ESP_SLEEP_WAKEUP_TIMER)
    {
      Serial.println("Directing panel..");
      pwm.begin();
      uint8_t mode1 = pwm.read8(PCA9685_MODE1);
      // Enter sleep so outputs stay off while we set neutral
      pwm.write8(PCA9685_MODE1, mode1 | PCA9685_MODE1_SLEEP);
      pwm.setOscillatorFrequency(27000000);
      pwm.setPWMFreq(SERVO_FREQ);

      // Use last-known angles (RTC) for neutral; fall back to mid-range if unset
      float neutralAzDeg = isnan(rtc_azimuth) ? (servoConfig.azMinDeg + servoConfig.azMaxDeg) / 2.0f : rtc_azimuth;
      float neutralElDeg = isnan(rtc_elevation) ? (servoConfig.elMinDeg + servoConfig.elMaxDeg) / 2.0f : rtc_elevation;
      uint16_t azNeutral = mapAzimuthToPulse(neutralAzDeg);
      uint16_t elNeutral = mapElevationToPulse(neutralElDeg);
      pwm.setPWM(servoConfig.azimuthChannel, 0, azNeutral);
      pwm.setPWM(servoConfig.elevationChannel, 0, elNeutral);

      // Power the booster while outputs are still disabled
      digitalWrite(ENPin, HIGH);
      delay(400);

      // Wake PCA9685 (enable outputs) and allow oscillator to settle
      pwm.write8(PCA9685_MODE1, (mode1 | PCA9685_MODE1_AI) & ~PCA9685_MODE1_SLEEP);
      delayMicroseconds(600);

      uint16_t azPulse = mapAzimuthToPulse(azimuth);
      uint16_t elPulse = mapElevationToPulse(elevation);

      Serial.print("Setting Az channel ");
      Serial.print(servoConfig.azimuthChannel);
      Serial.print(" to pulse ");
      Serial.println(azPulse);

      pwm.setPWM(servoConfig.azimuthChannel, 0, azPulse);
      delay(1000); // One at a time to limit current draw
      // Turn off azimuth and elevation servos power
      pwm.setPWM(servoConfig.azimuthChannel, 0, 0);
      delay(400);

      Serial.print("Setting El channel ");
      Serial.print(servoConfig.elevationChannel);
      Serial.print(" to pulse ");
      Serial.println(elPulse);

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

    send_data(azimuth, elevation);

    digitalWrite(TFT_BACKLIGHT, LOW);
  }
  else
  {
    Serial.println("No valid time received, skipping servo update.");
  }

  // --- DEEP SLEEP PREPARATION (LAST THING TO RUN) ---
  Serial.println("Entering deep sleep...");

  go2sleep(SLEEP_MINUTES * 60);
  // Should never reach here
  Serial.println("ERROR: Failed to enter deep sleep!");
}

void loop()
{
  // Not used; everything is in setup()
}