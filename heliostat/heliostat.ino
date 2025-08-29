// Heliostat controller with deep sleep (ESP32)

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "config.h" // Adafruit IO credentials
#include <AdafruitIO_WiFi.h>
#include <esp_sleep.h>

// --- Tuning constants ---
struct ServoConfig {
    uint8_t azimuthChannel = 0;
    uint16_t azMin = 150; // 0 deg (due west)
    uint16_t azMax = 460; // ~170 deg (due east)
    float azRangeDeg = 170.0;

    uint8_t elevationChannel = 1;
    uint16_t elMin = 370; // 0 deg (horizontal)
    uint16_t elMax = 490; // ~45 deg
    float elRangeDeg = 45.0;
};
ServoConfig servoConfig;

// Location for Malmö, SE
float latitude = 55.6;  // degrees
float longitude = 13.0; // degrees

#define SERVO_FREQ 50 // Hz

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// --- Adafruit IO Time Subscription ---
AdafruitIO_Time *iso = io.time(AIO_TIME_ISO);

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
    Serial.print("ISO Feed: ");
    Serial.println(data);
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

    float sunCenter = DEG_TO_RAD *
                      ((1.9146 - 0.004847 * elapsedT) * sin(solarMeanAnomaly) + (0.019993 - 0.000101 * elapsedT) * sin(2 * solarMeanAnomaly) + 0.00029 * sin(3 * solarMeanAnomaly));

    float solarTrueAnomaly = solarMeanAnomaly + sunCenter;
    float equatorObliquity = DEG_TO_RAD * (23 + 26 / 60. + 21.448 / 3600. - 46.815 / 3600 * elapsedT);

    long JDx = JD_whole - Y2K_JULIAN_DAY;
    float GreenwichHourAngle = 280.46061837 + (360 * JDx) % 360 + .98564736629 * JDx + 360.98564736629 * JD_frac;
    GreenwichHourAngle = fmod(GreenwichHourAngle, 360.0);

    float solarTrueLongitude = fmod(sunCenter + solarLongitude, TWO_PI);

    float rightAscension = atan2(sin(solarTrueLongitude) * cos(equatorObliquity), cos(solarTrueLongitude));
    float Declination = asin(sin(equatorObliquity) * sin(solarTrueLongitude));
    float hourAngle = DEG_TO_RAD * GreenwichHourAngle + Longitude - rightAscension;

    azimuth_deg = (PI + atan2(sin(hourAngle),
                              cos(hourAngle) * sin(Latitude) - tan(Declination) * cos(Latitude))) *
                  RAD_TO_DEG;

    elevation_deg = asin(sin(Latitude) * sin(Declination) + cos(Latitude) * cos(Declination) * cos(hourAngle)) * RAD_TO_DEG;
}

// --- Map degrees to servo pulse ---
uint16_t mapAzimuthToPulse(float azDeg)
{
    azDeg = constrain(azDeg, 0, servoConfig.azRangeDeg);
    return map(azDeg, 0, servoConfig.azRangeDeg, servoConfig.azMin, servoConfig.azMax);
}
uint16_t mapElevationToPulse(float elDeg)
{
    elDeg = constrain(elDeg, 0, servoConfig.elRangeDeg);
    return map(elDeg, 0, servoConfig.elRangeDeg, servoConfig.elMin, servoConfig.elMax);
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Heliostat waking up...");

    // Adafruit IO setup
    io.connect();
    iso->onMessage(handleISO);
    Serial.println(WIFI_SSID);
    int tries=0;
    
    while(io.status() < AIO_CONNECTED) {
        Serial.print(tries++);
        Serial.println(io.statusText());
        delay(500);
    }
    
    Serial.println();
    Serial.println(io.statusText());
    delay(10);

    // Wait for ISO time to arrive
    unsigned long start = millis();
    while(latestTime < 1000000000 && millis() - start < 10000) { // wait max 10s
        io.run();
        Serial.println("io.run");
        delay(100);
    }

    if (latestTime > 1000000000) {
        //start the servos if you got time
        pwm.begin();
        pwm.setOscillatorFrequency(27000000);
        pwm.setPWMFreq(SERVO_FREQ);

        float azimuth, elevation;
        calcSolarAzEl(latestTime, latitude, longitude, azimuth, elevation);

        uint16_t azPulse = mapAzimuthToPulse(azimuth);
        uint16_t elPulse = mapElevationToPulse(elevation);

        pwm.setPWM(servoConfig.azimuthChannel, 0, azPulse);
        pwm.setPWM(servoConfig.elevationChannel, 0, elPulse);
        delay(1000);
        Serial.print("Azimuth: ");
        Serial.print(azimuth);
        Serial.print(" deg, pulse: ");
        Serial.println(azPulse);
        Serial.print("Elevation: ");
        Serial.print(elevation);
        Serial.print(" deg, pulse: ");
        Serial.println(elPulse);
    } else {
        Serial.println("No valid time received.");
    }

    Serial.println("Sleeping for 1 minutes...");
    // Turn off azimuth and elevation servos
    pwm.setPWM(servoConfig.azimuthChannel, 0, 0);
    pwm.setPWM(servoConfig.elevationChannel, 0, 0);
    esp_deep_sleep(1 * 60 * 1000000ULL); // 10 minutes in microseconds
}

void loop()
{
    // Not used; everything is in setup()
}