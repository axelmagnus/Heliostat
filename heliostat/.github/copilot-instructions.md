# Heliostat Project - AI Coding Agent Instructions

## Project Overview
Solar tracking system for ESP32/ESP32-S3 boards with three distinct programs:
- **heliostat.ino**: Main sun-tracking heliostat with dual-axis servo control, solar position calculations, and deep sleep power management
- **static_panel/static_panel.ino**: Static solar panel monitor with current/voltage sensing and optional e-paper display
- **drain/drain.ino**: Battery drain testing utility with TFT display for power consumption analysis

## Architecture & Critical Patterns

### Hardware Configuration
- **ESP32-S3** microcontroller with deep sleep capability
- **Adafruit PCA9685 PWM driver** (I2C address 0x41) for servo control
- **INA219** current/voltage sensor for power monitoring
- **GPIO 13 (ENPin)** controls power boost converter AND onboard LED - requires special handling to prevent startup glitches
- All programs use **Adafruit IO WiFi** for cloud telemetry (credentials in `config.h`)

### Critical GPIO 13 Startup Glitch Workaround
The heliostat uses an aggressive pin stabilization strategy to prevent GPIO 13 from briefly going HIGH during ESP32 boot:
```cpp
// Constructor runs BEFORE setup() to lock pin LOW immediately
void earlyPinStabilization() __attribute__((constructor));
void earlyPinStabilization() {
  gpio_set_direction((gpio_num_t)ENPin, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)ENPin, 0);
  gpio_hold_en((gpio_num_t)ENPin); // Hardware hold during boot
}
```
In `setup()`, ALWAYS release hold, reconfigure pin, then re-enable hold before deep sleep:
```cpp
gpio_deep_sleep_hold_dis(); 
gpio_hold_dis((gpio_num_t)ENPin);
pinMode(ENPin, OUTPUT);
digitalWrite(ENPin, LOW);
// ... do work ...
gpio_hold_en((gpio_num_t)ENPin); // Re-enable before sleep
```

### Solar Position Calculation
`heliostat.ino` implements complete astronomical calculations from scratch:
- Julian date conversion for arbitrary timestamps
- Solar declination, right ascension, hour angle calculations
- Location: Malmö, SE (55.6°N, 13.0°E) - hardcoded in `latitude`/`longitude` globals
- Returns azimuth (0-360°) and elevation (-90 to +90°) for servo positioning

### Servo Control Pattern
Two-axis system with inverted azimuth mapping (higher degrees = lower pulse):
```cpp
ServoConfig servoConfig = {
  azimuthChannel: 0,   azMinDeg: 80°,  azMaxDeg: 280°,  
  azMinPulse: 480,     azMaxPulse: 100,  // Note: inverted mapping
  elevationChannel: 1, elMinDeg: 30°,   elMaxDeg: 90°,
  elMinPulse: 480,     elMaxPulse: 370
};
```
Power management: Enable boost → move azimuth → delay → disable azimuth PWM → move elevation → disable elevation PWM → disable boost (limits current draw)

### Deep Sleep Cycle
Both heliostat and static panel use 10-minute deep sleep cycles (`SLEEP_MINUTES`):
1. Wake → collect sensor data
2. Connect WiFi → fetch NTP time via Adafruit IO (`AdafruitIO_Time`)
3. Calculate/report position (heliostat) or monitor power (static_panel)
4. Publish to Adafruit IO group "heliostat" (prefix `stat_` for static panel fields)
5. Enter deep sleep for 10 minutes

### Current Measurement Approaches
Three methods are explored (blip counting is experimental, INA219 is primary):
- **INA219 sensor**: 500-sample averaging for stable current readings
- **Blip counting (experimental)**: Interrupt-based pulse counting on CHRG pin for low-power current estimation
- **ADC voltage sampling**: 500-sample averaging on analog pins

### Configuration Files
`config.h` in root and `static_panel/` contain WiFi credentials and Adafruit IO keys. These are committed (not .gitignored) - handle sensitively when sharing code.

## Development Workflow

### Building & Uploading
Use Arduino IDE or PlatformIO with ESP32 board definitions:
- Select board: **ESP32-S3** or **ESP32 Dev Module**
- Upload speed: 921600 baud recommended
- Partition scheme: Default 4MB with appropriate flash configuration

### Serial Debugging
All programs use **115200 baud** serial for debugging. Key telemetry includes:
- Sensor readings (voltage, current, blip counts)
- WiFi connection status (blink codes on GPIO 13/LED)
- Solar position calculations (azimuth/elevation)
- Deep sleep countdown

### Testing Without Hardware
- **Time simulation**: Modify `latestTime` directly instead of waiting for NTP
- **Servo calibration**: Adjust `ServoConfig` pulse ranges while observing physical movement
- **Power testing**: Use `drain.ino` with intentionally wrong WiFi credentials to burn power and measure discharge curves

## Common Modifications

### Changing Location
Update `latitude` and `longitude` globals in `heliostat.ino` (decimal degrees):
```cpp
float latitude = 55.6;   // Your latitude
float longitude = 13.0;  // Your longitude
```

### Tuning Servo Range
Physical stops vary by mount. Adjust `ServoConfig` pulse values in `heliostat.ino`:
- Test extreme positions incrementally to avoid hardware damage
- Azimuth has inverted mapping (higher angle = lower pulse)

### Adding Telemetry Fields
Use Adafruit IO group publishing pattern:
```cpp
group->set("field_name", value);
group->save();  // Publishes all set fields atomically
```

### Sleep Duration
Change `SLEEP_MINUTES` constant (currently 10) for different wake intervals. Balance power savings vs. tracking accuracy.

## Code Style Conventions
- C++/Arduino idioms with `.ino` extension for Arduino IDE compatibility
- Globals declared at file top (azimuth, elevation, sensor objects)
- Magic numbers defined as `#define` constants at top of file
- Heavy use of Adafruit libraries (IO, sensor drivers, display drivers)
- Serial output used extensively for debugging (not production-silent)
