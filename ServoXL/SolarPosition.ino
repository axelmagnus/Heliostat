// SolarPositionSimple.cpp
// Minimal solar azimuth/elevation calculation for given time and coordinates
// Based on Ken Willmott's SolarPosition library

#include <math.h>
#include <time.h>

// Constants
#define DEG_TO_RAD 0.017453292519943295
#define RAD_TO_DEG 57.29577951308232
#define PI 3.14159265358979323846
#define TWO_PI (2.0 * PI)

// Utility: Compute Julian Date
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

// Utility: Break time_t into date/time components (UTC)
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

// Main function: Calculate azimuth and elevation
void calcSolarAzEl(time_t t, float latitude_deg, float longitude_deg, float &azimuth_deg, float &elevation_deg)
{
    // Convert to radians
    float Latitude = latitude_deg * DEG_TO_RAD;
    float Longitude = longitude_deg * DEG_TO_RAD;

    // Constants
    const float DAYS_PER_JULIAN_CENTURY = 36525.0;
    const long Y2K_JULIAN_DAY = 2451545;

    // Break time
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

    // Azimuth (deg, east from north)
    azimuth_deg = (PI + atan2(sin(hourAngle),
                              cos(hourAngle) * sin(Latitude) - tan(Declination) * cos(Latitude))) *
                  RAD_TO_DEG;

    // Elevation (deg above horizon)
    elevation_deg = asin(sin(Latitude) * sin(Declination) + cos(Latitude) * cos(Declination) * cos(hourAngle)) * RAD_TO_DEG;
}

// Example usage
/*
#include <stdio.h>
int main() {
    time_t now = time(NULL);
    float lat = 40.0; // latitude in degrees
    float lon = -74.0; // longitude in degrees
    float az, el;
    calcSolarAzEl(now, lat, lon, az, el);
    printf("Azimuth: %f deg, Elevation: %f deg\n", az, el);
    return 0;
}
*/