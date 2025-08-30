// Heliostat controller with deep sleep (ESP32)

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include "config.h" // Adafruit IO credentials
#include <AdafruitIO_WiFi.h>
#include <esp_sleep.h>
#include <Adafruit_INA219.h>

// --- Tuning constants ---
struct ServoConfig
{
    uint8_t azimuthChannel = 0;
    float azMinDeg = 80.0;     // Minimum azimuth in degrees (e.g., east)
    float azMaxDeg = 280.0;    // Maximum azimuth in degrees (e.g., west)
    uint16_t azMinPulse = 480; // Pulse for azMinDeg
    uint16_t azMaxPulse = 100; // Pulse for azMaxDeg

    uint8_t elevationChannel = 1;
    float elMinDeg = 30.0;     // Minimum elevation in degrees (from horizon)
    float elMaxDeg = 90.0;     // Maximum elevation in degrees (straight up)
    uint16_t elMinPulse = 480; // Pulse for elMinDeg
    uint16_t elMaxPulse = 370; // Pulse for elMaxDeg
};
ServoConfig servoConfig;

// Location for Malmö, SE
float latitude = 55.6;  // degrees
float longitude = 13.0; // degrees

float busVoltage;
float current;

#define SERVO_FREQ 50 // Hz
int ENPin = 15;       // to shut off the booster

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// --- Adafruit IO Time Subscription ---
AdafruitIO_Time *iso = io.time(AIO_TIME_ISO);
// set up the group
AdafruitIO_Group *group = io.group("heliostat");

Adafruit_INA219 ina219;

volatile time_t latestTime = 0;

// Parse ISO-8601 string to time_t (UTC)
time_t parseISO8601(const char *isoStr)
{
    int year, month, day, hour, minute, second;
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
        Serial.print("Parsed Time: ");
        Serial.print(hour);
        Serial.print(":");
        Serial.println(minute);

        return mktime(&t);
    }
    return 0;
}

// ISO time callback
void handleISO(char *data, uint16_t len)
{
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
void collectData()
{
    Serial.println("Collect and send data.");
    // Measure data from the INA219 and send to Adafruit IO using group publish
    if (!ina219.begin())
    {
        Serial.println("Failed to find INA219 chip");
    }
    delay(200);
    busVoltage = ina219.getBusVoltage_V();
    current = 0;
    Serial.println(millis());
    // Sample the current 500 times and make an average
    for (int i = 0; i < 500; i++)
    {
        current += ina219.getCurrent_mA();
    }
    current /= 500;
    Serial.println(millis());

    Serial.print("Bus Voltage: ");
    Serial.print(busVoltage);
    Serial.println(" V");
    Serial.print("Current: ");
    Serial.print(current);
    Serial.println(" mA");
    float power = busVoltage * current; // in mWatts
}

void send_data(float azi, float elv)
{
    // Send data to Adafruit IO using group publish
    group->set("bus_voltage", busVoltage);
    group->set("current", current);
    group->set("elevation", elv);
    group->set("azimuth", azi);
    group->save();
    io.run();
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Collecting data..");
    collectData();
    pinMode(ENPin, OUTPUT);

    // Adafruit IO setup
    io.connect();
    iso->onMessage(handleISO);
    Serial.println(WIFI_SSID);
    int tries = 0;

    while (io.status() < AIO_CONNECTED)
    {
        Serial.print(tries++);
        Serial.println(io.statusText());
        delay(500);
    }

    Serial.println();
    Serial.println(io.statusText());
    delay(10);

    // Wait for ISO time to arrive
    unsigned long start = millis();
    while (latestTime < 1000000000 && millis() - start < 10000)
    { // wait max 10s
        io.run();
        Serial.println("io.run");
        delay(100);
    }

    if (latestTime > 1000000000) // Got time, get direction
    {
        float azimuth, elevation;
        calcSolarAzEl(latestTime, latitude, longitude, azimuth, elevation);
        azimuth -= 180; // to get zero at due south
        uint16_t azPulse = mapAzimuthToPulse(azimuth);
        uint16_t elPulse = mapElevationToPulse(elevation);

        // Only cahnge the panel if elevation is above 0, that is, the sun is up
        if (elevation > 0)
        {
            Serial.println("Directing panel..");
            // Enable power boost
            digitalWrite(ENPin, HIGH);
            delay(500);
            // start the servos if you got time

            pwm.begin();
            pwm.setOscillatorFrequency(27000000);
            pwm.setPWMFreq(SERVO_FREQ);

            pwm.setPWM(servoConfig.azimuthChannel, 0, azPulse);
            pwm.setPWM(servoConfig.elevationChannel, 0, elPulse);
            delay(2000);
            // Turn off azimuth and elevation servos
            pwm.setPWM(servoConfig.azimuthChannel, 0, 0);
            pwm.setPWM(servoConfig.elevationChannel, 0, 0);
            delay(300);
            // Turn off boost before measuring
            digitalWrite(ENPin, LOW);
        }
        Serial.print("Azimuth: ");
        Serial.print(azimuth);
        Serial.print(" deg, pulse: ");
        Serial.println(azPulse);
        Serial.print("Elevation: ");
        Serial.print(elevation);
        Serial.print(" deg, pulse: ");
        Serial.println(elPulse);

        send_data(azimuth, elevation);
    }
    else
    {
        Serial.println("No valid time received.");
    }
    Serial.println("Sleeping for 10 minutes...");
    esp_deep_sleep(10 * 60 * 1000000ULL); // 10 minutes in microseconds
}

void loop()
{
    // Not used; everything is in setup()
}