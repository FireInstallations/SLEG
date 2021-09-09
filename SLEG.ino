/*
   connect VRX to A0
   connect VRY to A1
   servo 0 x
   servo 4 y
    -SCL=A5
    -SDA=A4

  Configure with AT Commands as described in https://www.instructables.com/AT-command-mode-of-HC-05-Bluetooth-module/

  with the program S201105_HC05_AT.ino
  to this end connect Pin 11 Arduino (TX) to RXD of HC05
  and connect Pin 10 Arduino (RX) to TXD of HC05

  works with Android App "Arduino Joystick"

  the name of our HC05 is SLEG
  the Baud rate is 9600
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(10, 11); // CONNECT BT RX PIN TO ARDUINO 11 PIN | CONNECT BT TX PIN TO ARDUINO 10 PIN
#define SERVOMIN 136
#define SERVOMAX 539
#define SERVOMIDDLE 337
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

int SValx;
int SValy;
int SPWMx = map(SValx, -100, 100, SERVOMIN, SERVOMAX);
int SPWMy = map(SValy, 0, 100, SERVOMIN + 30, SERVOMAX);

unsigned long buttonDelay;
int driveAngle = 0;
bool resetDriveAngle = false;
byte fwbb = 1; // values have to be substracet by 1. -1 -> bus moves backwards. 0 -> bus stands. 1 -> bus moves forwads.

void setup() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  SValx = 0;
  SValy = 0;
  SPWMx = map(SValx, -100, 100, SERVOMIN, SERVOMAX);
  SPWMy = map(SValy, 0, 100, SERVOMIN + 30, SERVOMAX);

  BTSerial.begin(9600);  // HC-05 speed set in AT command mode
  buttonDelay = millis(); // wait 200ms for next button value
  pinMode(8, OUTPUT); // forward / backward
  pinMode(12, OUTPUT); //move / stop
  delay(200);
}
void loop() {
  if (BTSerial.available()) {
    String val = BTSerial.readStringUntil('#');
    //Serial.println(BTSerial.read());
    // cited from the description in "Arduino Joystick" App
    if (val.length() == 7) {
      int armAngle = val.substring(0, 3).toInt();
      byte strength = val.substring(3, 6).toInt();
      byte button = val.substring(6, 8).toInt();

      if ((button != 0) && (millis() - buttonDelay > 200) ) {
        HandleButton(button);
        buttonDelay = millis();
      }

      HandleJoystick (armAngle, strength);
    }
    Serial.flush();
    val = "";
  } else {
    float x = 0.95 * SPWMx + 0.05 * map(0, 100, -100, SERVOMIN, SERVOMAX);
    SPWMx = round(x);
    pwm.setPWM(0, 0, SPWMx);

    float y = 0.95 * SPWMy + 0.05 * map(0, 0, 100, SERVOMIN + 30, SERVOMAX);
    SPWMy = round(y);
    pwm.setPWM(4, 0, SPWMy);

    // slowly return drivement direction back to 0.
    if ((resetDriveAngle) && (driveAngle != 0)) {
      driveAngle *= 0.9;
      driveAngle = (int)(driveAngle / 10.0) * 10;

      int SdriveAngle = round(map(driveAngle, -90, 90, SERVOMIN, SERVOMAX));
      pwm.setPWM(8, 0, SdriveAngle);

      Serial.println("driveAngle: " + (String)driveAngle);
    }
  }

  delay(50);
}

void HandleButton(int buttonVal) {
  int SdriveAngle; // int output of drivement angle

  switch (buttonVal) {
    case 1: //left
      if (driveAngle >= 0) { //increase angle if last angle was 0 or also a left one
        driveAngle += 10;
        resetDriveAngle = false;
      } else
        resetDriveAngle = true;

      driveAngle = min(90, driveAngle); //overflow

      SdriveAngle = round(map(driveAngle, -90, 90, SERVOMIN, SERVOMAX));
      pwm.setPWM(8, 0, SdriveAngle);

      Serial.println("driveAngle: " + (String)driveAngle);
      break;
    case 2: //backward
      if (fwbb != 2) { //if not lat value was forward
        fwbb = 0; //set movemet to backward
        digitalWrite(8, LOW);
        digitalWrite(12, HIGH); //start moving
      } else {
        fwbb = 1; // else stop
        digitalWrite(12, LOW);
      }

      Serial.println("fwbb: " + (String)(fwbb - 1));
      break;
    case 3: // right
      if (driveAngle <= 0) { //decrease angle if last angle was 0 or also a right one
        driveAngle -= 10;
        resetDriveAngle = false;
      } else
        resetDriveAngle = true; //else reset to 0

      driveAngle = max(-90, driveAngle); //overflow

      SdriveAngle = round(map(driveAngle, -90, 90, SERVOMIN, SERVOMAX));
      pwm.setPWM(8, 0, SdriveAngle);

      Serial.println("driveAngle: " + (String)driveAngle);

      break;
    case 4: //foreward
      if (fwbb != 0) { //if last value was not backward (stop or forward)
        fwbb = 2; //set movement to forward
        digitalWrite(8, HIGH);
        digitalWrite(12, HIGH); //start moving
      } else {
        fwbb = 1; //else stop
        digitalWrite(12, LOW);
      }

      Serial.println("fwbb: " + (String)(fwbb - 1));
      break;
  }
}

void HandleJoystick (int armAngle, byte armspeed) {
  // compute x and y
  SValx = armspeed * cos(float(armAngle * PI / 180));
  SValy = armspeed * sin(float(armAngle * PI / 180));
  if (SValy < 0) SValy = 0;

  float x = 0.9 * SPWMx + 0.1 * map(SValx, 100, -100, SERVOMIN, SERVOMAX);

  SPWMx = round(min(x, SERVOMAX - 60)); //don't hit the screw
  pwm.setPWM(0, 0, SPWMx);

  float y = 0.9 * SPWMy + 0.1 * map(SValy, 0, 100, SERVOMIN + 30, SERVOMAX);
  SPWMy = round(y);
  pwm.setPWM(4, 0, SPWMy);

  Serial.println("x= " + (String)x + "   y= " + (String)y);

}
