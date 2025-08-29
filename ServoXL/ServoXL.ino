// servo code to be used with solar tracker
// This code is for testing the Adafruit PCA9685 16-channel PWM/Servo driver
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

//XL: Azi min:150 max:460=0-170 degrees, Eli:370-490=0-45 degrees

#define SERVOMIN  370 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  490 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

// our servo # counter
uint8_t servonum = 0;
int ENPin = 15;        // to shut off the booster


void setup() {
  Serial.begin(115200);
  Serial.println("8 channel Servo test!");
  pinMode(ENPin, OUTPUT);
  digitalWrite(ENPin,HIGH);
  delay(300);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates

  delay(10);
}


void loop() {
  // Drive each servo one at a time using setPWM()
  Serial.println(servonum);
  for (uint16_t pulselen = SERVOMIN; pulselen < SERVOMAX; pulselen++) {
    pwm.setPWM(servonum, 0, map(pulselen,SERVOMIN,SERVOMAX,150,460));
    pwm.setPWM(1, 0, pulselen);
    delay(15);
    Serial.println(pulselen);
  }

  delay(400);
  for (uint16_t pulselen = SERVOMAX; pulselen > SERVOMIN; pulselen--) {
    pwm.setPWM(servonum, 0, map(pulselen,SERVOMIN,SERVOMAX,150,460));
    pwm.setPWM(1, 0, pulselen);
    delay(15);
    Serial.println(pulselen);
  }

  delay(500);

}
