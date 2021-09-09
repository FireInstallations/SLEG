#include <Servo.h>
const int servoPin1 = 8;
const int servoPin2 = 9;
const int joyPinH = 3;
const int joyPinV = 4;

boolean colliding = false;

int lastStep = 1;
int grad = 0;
int maxGrad = 45;

int servoVal;

Servo servo1;
Servo servo2;
void setup() {
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  Serial.begin(9600);
  servo1.write(90);
  delay(200);
}
void loop() {
  joyStick();
  //such();
}
void joyStick() {
  servoVal = analogRead(joyPinH);
  servoVal = map(servoVal, 0, 1023, 0, 180);
  servo1.write(servoVal);
  servoVal = analogRead(joyPinV);
  servoVal = map(servoVal, 0, 1023, 180, 0);
  servo2.write(servoVal);
  delay(35);
}
void test() {
  int in = analogRead(A0);
  delay(100);
  if (in > 500) {
    colliding = true;
  } else {
    colliding = false;
  }
}
void such() {
  test();
  int waitTime = 100;
  if (colliding == true) {
    return;
  }
  int mitte = 90;
  servo1.write(mitte + grad);
  lastStep = 1;
  test();
  if (colliding == true) {
    return;
  }
  lastStep = -1;
  delay(waitTime);
  grad += lastStep;
  if (grad >= maxGrad || grad <= -maxGrad) {
    grad = 0;
    lastStep *= -1;
  }
}

