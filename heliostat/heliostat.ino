#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "config.h"  // Adafruit IO credentials
#include <AdafruitIO_WiFi.h>
#include <driver/gpio.h> // Required for low-level GPIO control
#include <esp_sleep.h> // Required for deep sleep functions
#include <Adafruit_INA219.h>
// Added peripherals for external wake displays
#include <Adafruit_SHT4x.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "Adafruit_MAX1704X.h"

// Ensure TFT backlight pin is defined (guard against missing macro)
#ifndef TFT_BACKLIGHT
#define TFT_BACKLIGHT 45
#endif

// I2C power control pin for ESP32-S3 Reverse TFT
#ifndef TFT_I2C_POWER
#define TFT_I2C_POWER 21
#endif

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
int ENPin = 12;        // to shut off the booster (Connected to GPIO 12)

// Buttons on ESP32-S3 Reverse TFT (adjust if needed)
#ifndef BUTTON_D1
#define BUTTON_D1 1
#endif
#ifndef BUTTON_D2
#define BUTTON_D2 2
#endif



// Teal color scheme (same as static_panel)
#define COLOR_DARK_TEAL 0x0410
#define COLOR_MID_TEAL  0x4E9C
#define COLOR_LIGHT_TEAL 0xAF3D

// Location for Malmö, SE
float latitude = 55.6;   // degrees
float longitude = 13.0;  // degrees

float busVoltage;
float current;
float currentWifi; //measured after servo movment
float cellVoltage; // from MAX17048
float chargeRate;  // from MAX17048

// RTC persisted quick/accurate readings for fast EXT1 wake display
RTC_DATA_ATTR float rtc_busVoltage = 3.33f;
RTC_DATA_ATTR float rtc_current = 0.0f;         // Accurate 500-sample current
RTC_DATA_ATTR float rtc_temperature = 0.0f;
RTC_DATA_ATTR float rtc_humidity = 0.0f;
RTC_DATA_ATTR float rtc_battPercent = 4.0f;
RTC_DATA_ATTR float rtc_cellVoltage = 3.7f;
RTC_DATA_ATTR float rtc_chargeRate = 0.0f;
RTC_DATA_ATTR uint64_t rtc_remaining_sleep_us = 0; // remaining until next timer wake
RTC_DATA_ATTR struct timeval rtc_sleep_enter_time = {0, 0}; // time when deep sleep entered

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
void renderSHT41(float temperature, float humidity);
void renderBattery(float percent, float cellV, float busV, float currmA);

// Fast render of stored SHT41 values (no sensor I/O)
void renderSHT41(float temperature, float humidity) {
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);
  tft.init(135, 240);
  tft.setRotation(1);
  tft.fillScreen(COLOR_DARK_TEAL);
  tft.fillRoundRect(10, 20, 215, 45, 10, COLOR_MID_TEAL);
  tft.fillRoundRect(10, 80, 215, 45, 10, COLOR_MID_TEAL);
  tft.setTextColor(COLOR_LIGHT_TEAL);
  tft.setTextSize(2); tft.setCursor(15, 25); tft.print("Temp");
  tft.setTextSize(3); tft.setCursor(120, 25); tft.print(isnan(temperature)?0:temperature,1); tft.setTextSize(2); tft.print(" C");
  tft.setTextSize(2); tft.setCursor(15, 85); tft.print("Humidity");
  tft.setTextSize(3); tft.setCursor(120, 85); tft.print(isnan(humidity)?0:humidity,0); tft.setTextSize(2); tft.print(" %");
}

// Fast render of stored battery / power values
void renderBattery(float percent, float cellV, float busV, float currmA) {
  pinMode(TFT_BACKLIGHT, OUTPUT);
  digitalWrite(TFT_BACKLIGHT, HIGH);
  tft.init(135, 240);
  tft.setRotation(1);
  tft.fillScreen(COLOR_DARK_TEAL);
  tft.fillRoundRect(5, 5, 110, 55, 8, COLOR_MID_TEAL);
  tft.fillRoundRect(125, 5, 110, 55, 8, COLOR_MID_TEAL);
  tft.fillRoundRect(5, 70, 110, 55, 8, COLOR_MID_TEAL);
  tft.fillRoundRect(125, 70, 110, 55, 8, COLOR_MID_TEAL);
  tft.setTextColor(COLOR_LIGHT_TEAL);
  tft.setTextSize(2); tft.setCursor(15, 15); tft.print("Batt %");
  tft.setTextSize(3); tft.setCursor(15, 35); tft.print(isnan(percent)?0:percent,0); tft.setTextSize(2); tft.print(" %");
  tft.setTextSize(2); tft.setCursor(135, 15); tft.print("Cell V");
  tft.setTextSize(3); tft.setCursor(135, 35); tft.print(isnan(cellV)?busV:cellV,2); tft.setTextSize(2); tft.print(" V");
  tft.setTextSize(2); tft.setCursor(15, 80); tft.print("Bus V");
  tft.setTextSize(3); tft.setCursor(15, 100); tft.print(busV,2); tft.setTextSize(2); tft.print(" V");
  tft.setTextSize(2); tft.setCursor(135, 80); tft.print("Current");
  tft.setTextSize(3); tft.setCursor(135, 100); tft.print(currmA,0); tft.setTextSize(2); tft.print(" mA");
}



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
  uint16_t elMaxPulse = 370;  // Pulse for elMaxPulse
};
ServoConfig servoConfig;
volatile time_t latestTime = 0;

// Parse ISO-8601 string to time_t (UTC)
time_t parseISO8601(const char *isoStr) {
  int year, month, day, hour, minute, second;
  // Use T%2d to match the 'T' separator and the following hour digits
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


// Quick one-shot reads for temp/humidity + battery percent
void readSensorsQuick() {
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
  if (!isnan(te.temperature)) {
    rtc_temperature = te.temperature;
    rtc_humidity = he.relative_humidity;
    Serial.println("SHT41 read OK");
  } else {
    Serial.println("SHT41 read invalid, keeping cached values");
  }
  
  // Fuel gauge - wait for device ready with timeout
  Serial.println("Reading MAX17048...");
  int maxRetries = 10;
  bool maxReady = false;
  for (int i = 0; i < maxRetries; i++) {
    if (maxlipo.isDeviceReady()) {
      maxReady = true;
      break;
    }
    delay(50);
  }
  
  if (maxReady) {
    rtc_cellVoltage = maxlipo.cellVoltage();
    rtc_battPercent = maxlipo.cellPercent();
    rtc_chargeRate = maxlipo.chargeRate();
    Serial.println("MAX17048 ready and read OK");
  } else {
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
  
  Serial.print("Temp: "); Serial.print(rtc_temperature); Serial.print(" C, Humidity: "); Serial.print(rtc_humidity); Serial.println(" %");
  Serial.print("Battery: "); Serial.print(rtc_battPercent); Serial.print(" %, Cell V: "); Serial.print(rtc_cellVoltage); Serial.println(" V");
}

// Accurate current averaging (500 samples) only for reset wake
void readAccurateCurrent() {
  if (!ina219.begin()) return;
  float acc = 0;
  for (int i = 0; i < 500; i++) {
    acc += ina219.getCurrent_mA();
  }
  rtc_current = acc / 500.0f;
  Serial.print("Accurate 500-sample current: ");
  Serial.println(rtc_current);
}

// Prepare pins, wake sources, compute remaining sleep, and enter deep sleep
void go2sleep(int seconds) {
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


  // Compute remaining sleep similar to static_panel
  struct timeval now;
  gettimeofday(&now, NULL);
  uint64_t elapsed_us = 0;
  if (rtc_sleep_enter_time.tv_sec != 0) {
    elapsed_us = (uint64_t)(now.tv_sec - rtc_sleep_enter_time.tv_sec) * 1000000ULL +
                 (uint64_t)(now.tv_usec - rtc_sleep_enter_time.tv_usec);
  }

  if (rtc_remaining_sleep_us == 0) {
    rtc_remaining_sleep_us = (uint64_t)seconds * 1000000ULL;
  }

  if (elapsed_us > 0 && elapsed_us < rtc_remaining_sleep_us) {
    rtc_remaining_sleep_us -= elapsed_us;
  } else {
    rtc_remaining_sleep_us = (uint64_t)seconds * 1000000ULL;
  }

  uint64_t sleep_us = rtc_remaining_sleep_us;
  Serial.print("Sleeping for ");
  Serial.print(sleep_us / 1000000ULL);
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

void send_data(float azi, float elv) {
  // Send data to Adafruit IO using group publish - split into two batches to avoid overflow
  Serial.println("Sending data - diminished");
  
  // Batch 1: Power and position data
  group->set("current", rtc_current);
  group->set("elevation", elv);
  
  group->set("temperature", rtc_temperature);
  group->set("humidity", rtc_humidity);
  group->set("battery_percent", rtc_battPercent);
  group->set("cell_voltage", rtc_cellVoltage);
  group->set("charge_rate", rtc_chargeRate);
  group->save();
  
  // Run io.run() to transmit batch 2
  int sendStart = millis();
  while (millis() - sendStart < 3000) {
    io.run();
    Serial.println("sending"); // Print Adafruit IO connection status
    delay(100);
  }
  Serial.println("Data publish loop complete.");
}

// --- CRITICAL EARLY PIN STABILIZATION (Fixes GPIO 13 startup glitch) ---
// This function runs automatically BEFORE setup() using the __attribute__((constructor))
// to ensure ENPin (GPIO 12) is LOW immediately on power-up, reset, or wake.
void earlyPinStabilization() __attribute__((constructor));
void earlyPinStabilization() {
  // 1. Immediately configure the pin to OUTPUT and set level LOW.
  gpio_set_direction((gpio_num_t)ENPin, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)ENPin, 0); // 0 = LOW

  // 2. AGGRESSIVELY apply the hardware hold state now.
  // This locks the pin LOW right after setting the level, preventing external circuits (like the LED pull-up)
  // from briefly pulling it HIGH during the remainder of the boot process.
  gpio_hold_en((gpio_num_t)ENPin); 
}
// --- END EARLY STABILIZATION ---


void setup() {
  Serial.begin(115200);

  Serial.println("\n\n========================================");
  Serial.println("HELIOSTAT BOOT - DEBUG BUILD");
  Serial.println("========================================");
  
  // Log wake cause immediately
  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  Serial.print("Wake cause: ");
  switch(cause) {
    case ESP_SLEEP_WAKEUP_UNDEFINED: Serial.println("RESET/POWER_ON"); break;
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("EXT0"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("EXT1 (button)");digitalWrite(TFT_BACKLIGHT, HIGH); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("TIMER"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD: Serial.println("TOUCHPAD"); break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("ULP"); break;
    default: Serial.println("OTHER"); break;
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
  if (cause == ESP_SLEEP_WAKEUP_EXT1) {
  
    uint64_t status = esp_sleep_get_ext1_wakeup_status();
    bool d1_triggered = status & (1ULL << BUTTON_D1);
    bool d2_triggered = status & (1ULL << BUTTON_D2);
    
    if (d2_triggered) {
      Serial.println("Button D2 -> SHT41 cached screen");
      renderSHT41(rtc_temperature, rtc_humidity);
    } else if (d1_triggered) {
      Serial.println("Button D1 -> Battery cached screen");
      renderBattery(rtc_battPercent, rtc_cellVoltage, rtc_busVoltage, rtc_current);
    } else {
      renderSHT41(rtc_temperature, rtc_humidity);
    }

    // Keep screen on briefly, then sleep without running servos/AIO
    delay(4000);
    digitalWrite(TFT_BACKLIGHT, LOW);
    Serial.println("Sleeping after external wake display...");
    go2sleep(SLEEP_MINUTES * 60);
    return;
  }

  
  readSensorsQuick(); // SHT41 + fuel gauge cached
  // This shoudl be a reset wake, we may perform accurate current sampling
  readAccurateCurrent();
  renderBattery(rtc_battPercent, rtc_cellVoltage, rtc_busVoltage, rtc_current);
  delay(2000); // Show battery screen for 3s
  renderSHT41(rtc_temperature, rtc_humidity);
  delay(2000); // Show SHT41 screen for 2s

  // Turn off backlight to save power
  digitalWrite(TFT_BACKLIGHT, LOW);
  // If voltage is low (<3.3V), skip WiFi and go back to sleep
  if (rtc_cellVoltage < 3.3f) {
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

  while (io.status() < AIO_CONNECTED && tries < 20) {
    Serial.print(tries++);
    digitalWrite(13, tries % 2 == 0 ? HIGH : LOW); // Use ENPin/LED for status blink
    Serial.println(io.statusText());
    delay(500);
  }

  // Turn off LED after connecting
  digitalWrite(13, LOW);
  Serial.println();
  Serial.println(io.statusText());
  delay(10);

  // Wait for ISO time to arrive
  unsigned long start = millis();
  // wait max 10s or until we get a valid time (time_t is non-zero)
  while (latestTime == 0 && millis() - start < 10000) { 
    io.run();
    Serial.println(io.statusText());
    delay(100);
  }

  if (latestTime > 1000000000)  // Got internet and time, get direction, start the servos 
  {
    calcSolarAzEl(latestTime, latitude, longitude, azimuth, elevation);
    
    uint16_t azPulse = mapAzimuthToPulse(azimuth);
    uint16_t elPulse = mapElevationToPulse(elevation);
    
    // Only change the panel if elevation is above 0, that is, the sun is up
    
    if (elevation > 0) {
      Serial.println("Directing panel..");
      // Enable power boost
      digitalWrite(ENPin, HIGH);
      delay(500);
      pwm.begin();
      pwm.setOscillatorFrequency(27000000);
      pwm.setPWMFreq(SERVO_FREQ);

      pwm.setPWM(servoConfig.azimuthChannel, 0, azPulse);
      delay(1000);  // One at a time to limit current draw
      // Turn off azimuth and elevation servos power
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
    // Measure current after servo movement
    currentWifi = 0;
    for (int i = 0; i < 500; i++) {
      currentWifi += ina219.getCurrent_mA();
    }
    currentWifi /= 500;
    send_data(azimuth, elevation); 
  } else {
    Serial.println("No valid time received, skipping servo update.");
  }
  
  // --- DEEP SLEEP PREPARATION (LAST THING TO RUN) ---
  Serial.println("Entering deep sleep...");
  
  go2sleep(SLEEP_MINUTES * 60);
  // Should never reach here
  Serial.println("ERROR: Failed to enter deep sleep!");
}

void loop() {
  // Not used; everything is in setup()
}