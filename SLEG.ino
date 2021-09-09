/*
   connect VRX to A0
   connect VRY to A1
   servo 0 x
   servo 4 y
    -SCL=A5
    -SDA=A4
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN 136
#define SERVOMAX 539
#define SERVOMIDDLE 337
#define YStickMiddlex 487 // derived from experiment
#define YStickMiddley 531
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

int SValx;
int SValy;
int SPWMx = map(SValx, 0, 1023, SERVOMIN, SERVOMAX);
int SPWMy = map(SValy, YStickMiddley, 1023, SERVOMIN + 30, SERVOMAX);

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  SValx = analogRead(A0);
  SValy = analogRead(A1);
  SPWMx = map(SValx, 0, 1023, SERVOMIN, SERVOMAX);
  SPWMy = map(SValy, YStickMiddley, 1023, SERVOMIN + 30, SERVOMAX);
  delay(200);
}
void loop() {
  joyStick();
  Serial.println("x  " + (String)SValx + "   y  " + (String)SValy);
  delay(50);
}
void joyStick() {
  SValx = analogRead(A0);
  float x = 0.9 * SPWMx + 0.1 * map(SValx, 0, 1023, SERVOMIN, SERVOMAX);
  SPWMx = (int)x;
  pwm.setPWM(0, 0, SPWMx);
  SValy = analogRead(A1);
  if (SValy < YStickMiddley)
    SValy = YStickMiddley;
  float y = 0.9 * SPWMy + 0.1 * map(SValy, YStickMiddley, 1023, SERVOMIN + 30, SERVOMAX);
  SPWMy = (int)y;
  pwm.setPWM(4, 0, SPWMy);
}
