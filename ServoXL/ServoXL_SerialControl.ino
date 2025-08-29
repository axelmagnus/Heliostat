// ServoXL_SerialControl.ino
// Control azimuth/elevation servos via serial commands
// 1: increase azimuth, 2: decrease azimuth
// 3: increase elevation, 4: decrease elevation

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

struct ServoConfig
{
    uint8_t azimuthChannel = 0;
    uint16_t azMin = 90;  // ~190 deg (due west)
    uint16_t azMax = 500; // 0 deg (due east)
    float azRangeDeg = 190.0;

    uint8_t elevationChannel = 1;
    uint16_t elMin = 330; // 0 deg (horizontal)
    uint16_t elMax = 540; // ~60 deg
    float elRangeDeg = 60.0;
};
ServoConfig servoConfig;

#define SERVO_FREQ 50
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
int ENPin = 15;

uint16_t azPulse = 0; // Current azimuth pulse
uint16_t elPulse = 0; // Current elevation pulse

void moveServos()
{

    digitalWrite(ENPin, HIGH);
    delay(250);
    pwm.setPWM(servoConfig.azimuthChannel, 0, azPulse);
    pwm.setPWM(servoConfig.elevationChannel, 0, elPulse);
    Serial.print("Moved to Azimuth pulse: ");
    Serial.print(azPulse);
    Serial.print(", Elevation pulse: ");
    Serial.println(elPulse);

    digitalWrite(ENPin, LOW);
}

void setup()
{
    Serial.begin(115200);
    pinMode(ENPin, OUTPUT);
    Serial.println("ServoXL Serial Control");
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    delay(10);
    azPulse = servoConfig.azMin + 30;
    elPulse = servoConfig.elMin + 30;
    moveServos();
}

void loop()
{
    static String inputString = "";
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n' || inChar == '\r') {
            if (inputString.length() > 0) {
                uint16_t az = inputString.toInt();
                if (az >= servoConfig.azMin && az <= servoConfig.azMax) {
                    azPulse = az;
                    moveServos();
                } else {
                    Serial.print("Invalid azimuth pulse. Enter value between ");
                    Serial.print(servoConfig.azMin);
                    Serial.print(" and ");
                    Serial.println(servoConfig.azMax);
                }
                inputString = "";
            }
        } else {
            inputString += inChar;
        }
    }
    delay(20);
}
