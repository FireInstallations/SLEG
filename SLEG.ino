/*This program controls the SLEG via a bluetooth joystick (use it in landscape mode)
   it is equipped with 3 servos, two for the arm (panthograph) control and one for directing the bus
   the bus electronics is removed, forward and backward movement is cotrolled by relays
   for details refer to the electronic scheme

   servo 0 x
   servo 4 y
   servo 8 direct
    -SCL=A5     (PCA9685)
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
#define SERVOMIDDLE 395
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

int SValx;
int SValxold;// Servo value derived from Joystick value for x (horizontal arm value)
int SValy;
int SValyold;// Servo value derived from Joystick value for y (vertical arm value)
// Servo x and y values for the PWM signal
int SPWMx;
int SPWMxold;
int SPWMy;
int SPWMyold;

unsigned long buttonDelay;
int SDirect = 0;// Servo value of bus direction
bool resetSDirect = false;
byte fwbw = 1; // values have to be substracet by 1. -1 -> bus moves backward. 0 -> bus stopps. 1 -> bus moves forward.

void setup() {
  pinMode(A6, INPUT); // 3:1 voltage devider
  pinMode(A7, INPUT); // accu voltage
  pinMode(8, OUTPUT); // forward low/ backward high
  pinMode(12, OUTPUT); // move low/ stop high
  digitalWrite(12, HIGH); // bus is stopped
  pinMode(13, OUTPUT); // if wired low
  digitalWrite(9, HIGH); // not wired
  pinMode(9, OUTPUT); // if load accu low
  digitalWrite(9, HIGH); // load is off
  pinMode(7, INPUT_PULLUP); // Pantograph too high, lower endswitch active, move down
  pinMode(6, INPUT_PULLUP); // Pantograph too far left,right endswitch active, move right
  pinMode(5, INPUT_PULLUP); // Pantograph too far right,left endswitch active, move left
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  SValx = 0;
  SValy = 0;
  SPWMx = SERVOMIDDLE;
  SPWMy = SERVOMIN + 30;
  SPWMxold = SPWMx;
  SPWMyold = SPWMy;

  BTSerial.begin(115200);  // HC-05 speed set in AT command mode
  buttonDelay = millis(); // wait 200ms for next button value

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
    // slowly return drive direction back to 0.
    if ((resetSDirect) && (SDirect != 0)) {
      SDirect *= 0.9;
      SDirect = (int)(SDirect / 10.0) * 10;

      int SPWMDirect = round(map(SDirect, -90, 90, SERVOMIN, SERVOMAX));
      pwm.setPWM(8, 0, SPWMDirect);

      //Serial.println("SDirect: " + (String)SDirect);
    }
  }

  Smove();
  //Serial.print((String)SPWMy + "    ");
  //Serial.println(SPWMyold);
  //delay(50);
}

void HandleButton(int buttonVal) {
  int SPWMDirect; // int output of drivement angle

  switch (buttonVal) {
    case 4: //left
      if (SDirect >= 0) { //increase angle if last angle was 0 or also a left one
        SDirect += 10;// increase by about 10 degrees
        resetSDirect = false;
      } else
        resetSDirect = true;

      SDirect = min(90, SDirect); //overflow

      SPWMDirect = round(map(SDirect, -90, 90, SERVOMIN, SERVOMAX));
      pwm.setPWM(8, 0, SPWMDirect);

      Serial.println("SDirect: " + (String)SDirect);
      break;
    case 1: //backward
      if (fwbw != 2) { //if not last value was forward
        fwbw = 0; //set movemet to backward
        digitalWrite(8, HIGH);
        digitalWrite(12, LOW); //start moving
      } else {
        fwbw = 1; // else stop
        digitalWrite(12, HIGH);
      }

      Serial.println("fwbw: " + (String)(fwbw - 1));
      break;
    case 2: // right
      if (SDirect <= 0) { //decrease angle if last angle was 0 or also a right one
        SDirect -= 10;// decrease by about 10 degrees
        resetSDirect = false;
      } else
        resetSDirect = true; //else reset to 0

      SDirect = max(-90, SDirect); //overflow

      SPWMDirect = round(map(SDirect, -90, 90, SERVOMIN, SERVOMAX));
      pwm.setPWM(8, 0, SPWMDirect);

      Serial.println("SDirect: " + (String)SDirect);

      break;
    case 3: //forward
      if (fwbw != 0) { //if last value was not backward (stop or forward)
        fwbw = 2; //set movement to forward
        digitalWrite(8, LOW);
        digitalWrite(12, LOW); //start moving
      } else {
        fwbw = 1; //else stop
        digitalWrite(12, HIGH);
      }

      Serial.println("fwbw: " + (String)(fwbw - 1));
      break;
  }
}

void HandleJoystick (int armAngle, byte armspeed) {
  // compute x and y
  SValx = armspeed * cos(float(armAngle * PI / 180));
  SValy = armspeed * sin(float(armAngle * PI / 180));
  //Serial.println("SValx= " + (String)SValx + "   SValy= " + (String)SValy);
  if (SValy < 0) SValy = 0;
  if (SValx != SValxold) {
    //Serial.println("xxx");
    if (SValx <= 0)
      SPWMx = map(SValx, 0, -100, SERVOMIDDLE, SERVOMAX);
    else
      SPWMx = map(SValx, 100, 0, SERVOMIN, SERVOMIDDLE);

    SValxold = SValx;
    //Serial.println("SPWMx= " + (String)SPWMx + "   SPWMy= " + (String)SPWMy);
  }
  if (SValy != SValyold) {


    SPWMy =  map(SValy, 0, 100, SERVOMIN + 30, SERVOMAX);
    SValyold = SValy;
  }
}

/*if ((resetSDirect) && (SDirect != 0)) {
      SDirect *= 0.9;
      SDirect = (int)(SDirect / 10.0) * 10;

      int SPWMDirect = round(map(SDirect, -90, 90, SERVOMIN, SERVOMAX));
      pwm.setPWM(8, 0, SPWMDirect);

      //Serial.println("SDirect: " + (String)SDirect);
    }*/

void Smove() {

  if (SPWMx > SPWMxold) {
    SPWMxold++;
    pwm.setPWM(0, 0, SPWMxold);
    delay(5);

  } else if (SPWMx < SPWMxold) {
    SPWMxold--;
    pwm.setPWM(0, 0, SPWMxold);
    delay(5);

  }
  if (SPWMy > SPWMyold) {
    SPWMyold++;
    pwm.setPWM(4, 0, SPWMyold);
    delay(3);

  } else if (SPWMy < SPWMyold) {
    SPWMyold--;
    pwm.setPWM(4, 0, SPWMyold);
    delay(3);

  }
}
