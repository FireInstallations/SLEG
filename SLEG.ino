/*This program controls the SLEG via a bluetooth joystick (use it in landscape mode)
   it is equipped with 3 servos, two for the arm control and one for directing the bus
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

float SValx;
float SValxold;// Servo value derived from Joystick value for x (horizontal arm value)
float SValy;
float SValyold;// Servo value derived from Joystick value for y (vertical arm value)
// Servo x and y values for the PWM signal
int SPWMx;
int SPWMxold;
int SPWMy;
int SPWMyold;

unsigned long buttonDelay, printDelay;
int SDirect = 0;// Servo value of bus direction
int SPWMDirect;
int SPWMDirectold;

// position of last wire connection
int XConnect, YConnect = 0;

bool AutoOverheadConnect = false;
// 255 = don't know where to connect and allredy beebed
// 0 = nothing
// 1 = start connecting moving to 1/2 YConnect
// 2 = move to XConnect
// 3 = move to full YConnect
// 4 =
byte OverheadConnectingState = 0;


byte fwbw = 1; // values have to be substracet by 1. -1 -> bus moves backward. 0 -> bus stopps. 1 -> bus moves forward.
void setup() {
  pinMode(A6, INPUT); // 3:1 voltage devider
  pinMode(A7, INPUT); // accu voltage
  pinMode(8, OUTPUT); // forward low/ backward high
  pinMode(12, OUTPUT); // move low/ stop high
  digitalWrite(12, HIGH); // bus is stopped
  pinMode(13, OUTPUT); // if wired low
  digitalWrite(13, HIGH); // not wired
  pinMode(9, OUTPUT); // if load accu low
  digitalWrite(9, HIGH); // load is off
  pinMode(7, INPUT_PULLUP); // Pantograph too high, lower endswitch active, move down
  pinMode(6, INPUT_PULLUP); // Pantograph too far left,right endswitch active, move right
  pinMode(5, INPUT_PULLUP); // Pantograph too far right,left endswitch active, move left
  pinMode(3, OUTPUT); // velocity control
  analogWrite(3, 70); // set velocity
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
  SPWMDirect = round(map(0, -90, 90, SERVOMIN, SERVOMAX));
  SPWMDirectold = SPWMDirect;

  BTSerial.begin(115200);  // HC-05 speed set in AT command mode
  buttonDelay = millis(); // wait 200ms for next button value
  printDelay = millis(); // toimer for printing

  delay(200);
}
void loop() {
  if (BTSerial.available()) {
    String val = BTSerial.readStringUntil('#');
    //Serial.println(BTSerial.read());
    // cited from the description in "Arduino Joystick" App
    if ((val.length() == 7) && (isValidNumber(val))) {
      int armAngle = val.substring(0, 3).toInt();
      byte strength = val.substring(3, 6).toInt();
      byte button = val.substring(6, 8).toInt();

      if ((button != 0) && (millis() - buttonDelay > 200) ) {
        HandleButton(button);
        buttonDelay = millis();
      }

      if ((button == 0) || !((OverheadConnectingState >= 4) && (OverheadConnectingState != 255)))
        HandleJoystick (armAngle, strength);

    }
    Serial.flush();
    val = "";

  }

  switch (getOverheadConnectorState ()) {// ToDo: don't overwrite user interaction
    case 0:
      if (SPWMyold <= SPWMy) { //ToDo: Endlagenschalter l/r beachten!
        XConnect = SPWMxold;
        YConnect = constrain(SPWMyold, SERVOMIN + 30, SERVOMAX);
      }
      break;

    case 2: // not connected
      if ((AutoOverheadConnect) && ((OverheadConnectingState > 0) && (OverheadConnectingState <= 30))) //should we try to connect?
        HandleOverheadConnection ();
      break;
  }
  
  if ((AutoOverheadConnect) && (OverheadConnectingState > 30) && (OverheadConnectingState != 255)) {
    HandleOverheadConnection ();
  }

  if (millis() - printDelay > 1000) {
    Serial.println("wired detected    " + (String)analogRead(A6));
    // Serial.println("Accu voltage    " + (String)analogRead(A7));
    Serial.println("too high   " + (String)digitalRead(6));// low active
    /* Serial.println("too far right   " + (String)digitalRead(5));// low active
      Serial.println("too far left   " + (String)digitalRead(7));// low active*/

    Serial.println("joytick Y: " + (String) map(SValyold * 10, 0, 1000, SERVOMIN + 30, SERVOMAX) );
    Serial.println("last connect coordinates " + (String)XConnect + "; " + (String)YConnect);

    Serial.println();
    printDelay = millis();
  }

  Smove();
}

void HandleOverheadConnection () {
  int TempY;

  switch (OverheadConnectingState) {
    case 1://start connecting by movint to 1/2 YConnect
      TempY = round((float)map(YConnect, SERVOMIN + 30, SERVOMAX, 0, 1000) / 2.0);
      SPWMy = map(TempY, 0, 1000, SERVOMIN + 30, SERVOMAX);

      if (abs(SPWMy - SPWMyold) == 0)  //if we readched it next state
        OverheadConnectingState = 2;

      break;

    case 2: //next step is moving to XConnect
      SPWMx = XConnect;

      if (abs(SPWMx - SPWMxold) == 0)
        OverheadConnectingState = 3; //if we readched it next state

      break;

    case 3: //moving fully to YConnect
      SPWMy = YConnect;

      if (abs(SPWMy - SPWMyold) == 0)
        OverheadConnectingState = 4; //if we readched it next state

      break; //if we are lucky our last coordinates do fit and we are connected now

    case 4: // cases 4->30 are reseved for help while connecting
      //Serial.println("help me!");
      break;

    /********************** -{ deconnecting }- **********************/
    case 31: //start deconnecting by movint to 1/2 YConnect
      TempY = round((float)map(SPWMyold, SERVOMIN + 30, SERVOMAX, 0, 1000) / 2.0);
      SPWMy = map(TempY, 0, 1000, SERVOMIN + 30, SERVOMAX);

      //PWMOld werte statt dessen verwenden!

      if (abs(SPWMy - SPWMyold) == 0) {  //if we readched it next state
        OverheadConnectingState = 32;

      }
      break;

    case 32:
      SPWMx = SERVOMIDDLE;

      if (abs(SPWMx - SPWMxold) == 0) {
        OverheadConnectingState = 33; //if we readched it next state
      }
      break;

    case 33:
      SPWMy = SERVOMIN + 30;
      if (abs(SPWMy - SPWMyold) == 0) {
        OverheadConnectingState = 0; //if we readched it default
        AutoOverheadConnect = false;
      }
      break;
  }
}


byte getOverheadConnectorState () {
  if ((analogRead(A6) > 735) && digitalRead(6)) {
    return 0; //connected
  } else if (analogRead(A6) > 700) {
    return 1; //to low
  } else
    return 2; //not connected
}

void HandleButton(int buttonVal) {

  switch (buttonVal) {
    case 4: //left
      if (SDirect >= 0) { //increase angle if last angle was 0 or also a left one
        SDirect += 10;// increase by about 10 degrees
      } else
        SDirect = 0;

      SDirect = min(90, SDirect); //overflow

      SPWMDirect = round(map(SDirect, -90, 90, SERVOMIN, SERVOMAX));
      break;
    case 1: //backward //ToDO resett beeb; deconnect
      if (fwbw != 2) { //if not last value was forward

        if (OverheadConnectingState != 0) {
          if (OverheadConnectingState == 255)
            OverheadConnectingState = 0;
          else {
            OverheadConnectingState = 31;
            Serial.println("start resett");
          }
        } else {

          fwbw = 0; //set movemet to backward
          digitalWrite(8, HIGH);
          digitalWrite(12, LOW); //start moving
        }
      } else {
        fwbw = 1; // else stop
        digitalWrite(12, HIGH);
        AutoOverheadConnect = false;
      }
      break;
    case 2: // right
      if (SDirect <= 0) { //decrease angle if last angle was 0 or also a right one
        SDirect -= 10;// decrease by about 10 degrees
      } else
        SDirect = 0; //else reset to 0

      SDirect = max(-90, SDirect); //overflow

      SPWMDirect = round(map(SDirect, -90, 90, SERVOMIN, SERVOMAX));

      break;
    case 3: //forward
      if (fwbw != 0) { //if last value was not backward (stop or forward)


        if (getOverheadConnectorState ()) { // if not connected
          if ((XConnect != 0) && (YConnect != 0)) { //if we know where to connect try to connect
            OverheadConnectingState = 1;
            AutoOverheadConnect = true;
          } else { //tell the user to connect manually, but only once

            if (OverheadConnectingState != 255) {
              digitalWrite(13, LOW);
              delay(1000);
              digitalWrite(13, HIGH);
              delay(10);
              OverheadConnectingState = 255;
            }
          }
        }

        // if connected or already warned that not start moving
        if ((!getOverheadConnectorState ()) || (OverheadConnectingState == 255)) {
          fwbw = 2; //set movement to forward
          digitalWrite(8, LOW);
          digitalWrite(12, LOW); //start moving
        }
      } else {
        fwbw = 1; //else stop
        digitalWrite(12, HIGH);
      }
      break;
  }
}

void HandleJoystick (int armAngle, byte armelongation) {
  // compute x and y
  SValx = armelongation * cos(float(armAngle * PI / 180));
  SValy = armelongation * sin(float(armAngle * PI / 180));

  if ((SValy <= 10) && (armAngle != 0)) return;// to avoid misssending from Joy Stick


  SValxold = 0.9 * SValxold + 0.1 * SValx;
  if (SValxold <= 0)
    SPWMx = map(SValxold * 10, 0, -1000, SERVOMIDDLE, SERVOMAX);
  else
    SPWMx = map(SValxold * 10, 1000, 0, SERVOMIN, SERVOMIDDLE);




  if (SValy < 0) SValy = 0;// use only upper half of Joystickpad

  if (SValy != 0) {
    // gliding avarage as far as not return to zero is adviced
    SValyold = 0.9 * SValyold + 0.1 * SValy;
  } else SValyold = SValy;
  SPWMy =  map(SValyold * 10, 0, 1000, SERVOMIN + 30, SERVOMAX);


}

void Smove() {
  //arm
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

    if (SPWMy - SPWMyold < 20)
      delay(3);
    if (SPWMy - SPWMyold < 10)
      delay(7);

  } else if (SPWMy < SPWMyold) {
    SPWMyold--;
    pwm.setPWM(4, 0, SPWMyold);
    delay(3);

    if (SPWMyold - SPWMy < 20)
      delay(3);
    if (SPWMyold - SPWMy < 10)
      delay(7);
  }

  //driving  direction
  if (SPWMDirect > SPWMDirectold ) {
    SPWMDirectold ++;
    pwm.setPWM(8, 0, SPWMDirectold);
    delay(1);

  } else if (SPWMDirect < SPWMDirectold) {
    SPWMDirectold --;
    pwm.setPWM(8, 0, SPWMDirectold);
    delay(1);
  }

}

//returns only true if every char in String is a digit
boolean isValidNumber(String str) {
  bool OnlyDigit = true;
  for (char c : str) {
    OnlyDigit = isDigit(c) && OnlyDigit;
  }
  return OnlyDigit;
}
