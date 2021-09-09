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
#define OFFSET 30
#define SERVOMAX 539
#define SERVOMIDDLE 395
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define SmoveDefaultDelay 6

float SValx;
float SValxold;// Servo value derived from Joystick value for x (horizontal arm value)
float SValy;
float SValyold;// Servo value derived from Joystick value for y (vertical arm value)
// Servo x and y values for the PWM signal
int SPWMx;
int SPWMxold;
int SPWMy;
int SPWMyold;
long SmoveDelayTime; //time countet since last call
byte SmoveMinDelay = SmoveDefaultDelay; //minimum of millisecound between two Smove calls

unsigned long printDelay;
int SDirect = 0;// Servo value of bus direction
int SPWMDirect;
int SPWMDirectold;

// position of last wire connection
int XConnect = 0, YConnect = 0;

int HomeProgress = 1; // Steps for autohoming
int AutoWireProgress = 0; // Steps for Auto Wiring
bool JoyStickEnable = true; // Enable and disable Joy Stick
bool ButtonEnable = true; // Enable and disable Button
bool BTEnable = true; // Enable Blue tooth
bool FoundPos = false; // Position found
bool HomePos = true; // Panthograph in home position
bool IsDriving = false; // are the weels moving?

/*
   JoysitckMode describes if the joystick moves the whole bus or the arm.
   true means the bus
   false means the arm
*/
bool JoystickMode = false;

void setup() {
  pinMode(A6, INPUT); // 3:1 voltage devider
  pinMode(A7, INPUT); // accu voltage
  pinMode(8, OUTPUT); // forward low/ backward high
  pinMode(12, OUTPUT); // move low/ stop high
  digitalWrite(12, HIGH); // bus is stopped
  pinMode(13, OUTPUT); // if wired low
  digitalWrite(13, HIGH); // not wired, no beep
  pinMode(9, OUTPUT); // if load accu low
  digitalWrite(9, HIGH); // load is off
  pinMode(7, INPUT_PULLUP); // Pantograph too high, lower endswitch active, move down
  pinMode(6, INPUT_PULLUP); // Pantograph too far left,right endswitch active, move right
  pinMode(5, INPUT_PULLUP); // Pantograph too far right,left endswitch active, move left
  pinMode(3, OUTPUT); // velocity control
  analogWrite(3, 80); // set velocity
  Serial.begin(9600);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  SValx = 0;
  SValy = 0;
  SPWMx = SERVOMIDDLE;
  SPWMy = SERVOMIN + OFFSET;
  SPWMxold = SPWMx;
  SPWMyold = SPWMy;
  SPWMDirect = round(map(0, -90, 90, SERVOMIN, SERVOMAX));
  SPWMDirectold = SPWMDirect;

  BTSerial.begin(115200);  // HC-05 speed set in AT command mode
  printDelay = millis(); // timer for printing
  SmoveDelayTime = millis();

  delay(200);
}
void loop() {
  if (BTSerial.available() && BTEnable) { //!!!disables button and joystick, control via app  disabled!
    String val = BTSerial.readStringUntil('#');
    // cited from the description in "Arduino Joystick" App
    if ((val.length() == 7) && (isValidNumber(val))) {
      int armAngle = val.substring(0, 3).toInt();
      byte strength = val.substring(3, 6).toInt();
      byte button = val.substring(6, 8).toInt();

      /* If button was pressed
         If joystick doesn't have a value or button is home
         If button was not blocked by other program parts (e.a. homing)
      */
      if ((button != 0) && ((strength == 0) || (button == 1) ) && (ButtonEnable) ) {
        HandleButton(button, strength);
      }

      if (JoyStickEnable)
        HandleJoystick (armAngle, strength);

    }
    Serial.flush();
    val = "";

  }

  if (millis() - printDelay > 1000) {

    Serial.println("wired detected    " + (String)analogRead(A6));
    Serial.println("Accu voltage    " + (String)analogRead(A7));
    Serial.println("too high   " + (String)digitalRead(6));// low active
    /* Serial.println("too far right   " + (String)digitalRead(5));// low active
      Serial.println("too far left   " + (String)digitalRead(7));// low active*/
    Serial.println("Wired State    " + (String)GetWiredState()  );
    Serial.println("joystick Y: " + (String)SPWMy  );
    Serial.println("last connect coordinates " + (String)XConnect + "; " + (String)YConnect);
    Serial.println("HomeProgress   " + (String)HomeProgress);
    Serial.println("JoystickMode: " + (String)JoystickMode);
    Serial.println("IsDriving: " + (String)IsDriving);
    Serial.println("GetWiredState(): " + (String)GetWiredState());

    Serial.println();
    printDelay = millis();
  }

  AutoWiring();

  // store values if properly connected and disable joystick
  if (GetWiredState() == 0) {
    XConnect = SPWMxold;
    YConnect = SPWMyold;
    FoundPos = true;
    if (!JoystickMode) // only disable the joystick if its currintly controling the arm.
      JoyStickEnable = false;
  }

  if (millis() - SmoveDelayTime >= SmoveMinDelay) {
    Smove();
    SmoveDelayTime = millis();
  }

  if (HomeProgress > 0)Homing();
}

void Homing() {
  switch (HomeProgress) {

    case 1: // start sequence move y
      BTEnable = false;

      SPWMy = 280;
      if (abs(SPWMy - SPWMyold) == 0)
        HomeProgress++;

      break;
    case 2: // move x home
      SPWMx = SERVOMIDDLE;

      if (abs(SPWMx - SPWMxold) == 0)
        HomeProgress++;

      break;
    case 3: // move y home and return to case 0
      SPWMy = SERVOMIN + OFFSET;

      if (abs(SPWMy - SPWMyold) == 0) {
        HomeProgress = 0;
        HomePos = true;
        BTEnable = true;

        Serial.flush();
      }
      break;
  }
}

void AutoWiring() {
  switch (AutoWireProgress) {
    case 0: // end of autowiring
      ButtonEnable = true;
      break;
    case 1: // start sequence move y
      JoyStickEnable = false;
      ButtonEnable = false;
      SPWMy = 260;
      if (abs(SPWMy - SPWMyold) == 0)
        AutoWireProgress++;
      break;
    case 2: // move x home
      SPWMx = XConnect;
      if (abs(SPWMx - SPWMxold) == 0)
        AutoWireProgress++;
      break;
    case 3: // move y home and return to case 0
      SPWMy = YConnect;
      if (abs(SPWMy - SPWMyold) == 0)
        AutoWireProgress++;
      break;
    case 4: // check if sufficiently wired
      delay(1000);
      AutoWireProgress = 0;
      if (GetWiredState() == 6)
        HomeProgress = 1;
      break;
  }
}


byte GetWiredState () {
  int voltage = analogRead(A6);
  bool toohigh = !digitalRead(6); // note that low means, that the switch is actuated
  bool tooleft = !digitalRead(7);
  bool tooright = !digitalRead(5);
  // the following logic enables to control search for contact and driving with contact

  
  if (voltage > 650)
    digitalWrite(13, LOW); //successfully connecet turn wire power on
  else
    digitalWrite(13, HIGH); //not connected
    

  if ((voltage > 700) || (IsDriving && (voltage > 650) )) {
    if (toohigh && !tooleft && !tooright)
      return 0;
    if (!toohigh && !tooleft && !tooright)
      return 1;
    if (tooright)
      return 2;
    if (tooleft)
      return 3;
    if (toohigh)
      return 4;

    return 5;

  } else
    return 6; // not connected
}

void HandleButton(int buttonVal, int joyStrength) {
  switch (buttonVal) {
    case 4: //"  ⃞  " joy arm
      SmoveMinDelay = SmoveDefaultDelay;

      if ((!IsDriving) && (joyStrength == 0)) {
        if (!HomePos) {
          HomeProgress = 1;
        }
        JoystickMode = false;
        JoyStickEnable = true;
      } // if we are currintly moving ignore all input.
      break;

    case 2: //" ◯ " joy drive
      if ((joyStrength == 0)) {  // if we are currintly moving ignore all input.

        if (GetWiredState() == 6) { // if not connected but also not home return home
          if (!HomePos)
            HomeProgress = 1;
        }

        JoystickMode = true;
        JoyStickEnable = true;
      }
      break;

    case 3: //" ⃤ " Connect wire if position of wire known, not connected and also not moving
      if ((!IsDriving) && (FoundPos) && (GetWiredState () == 6) )
        AutoWireProgress = 1;
      break;

    case 1: //" ╳ " deconnect //ToDo: Home manchmal kaputt
      HomeProgress = 1;
      break;
  }
}

void HandleJoystick (int armAngle, byte armelongation) {
  /*
     The Joystick dictates the movement of the hole bus (drive) OR
     of the wire connector arm.
     If JoystickMode happend to be true it is drive, if false arm.
  */

  // compute x and y
  SValx = armelongation * cos(float(armAngle * PI / 180));
  SValy = armelongation * sin(float(armAngle * PI / 180));

  IsDriving = false;

  if (JoystickMode) { // drive
    SPWMDirect = map(SValx, 100, -100, SERVOMIN, SERVOMAX);

    int velocity = map(abs(SValy), 0, 100, 60, 180);
    analogWrite(3, velocity); // set velocity

    if (velocity > 60)
      IsDriving = true;

    if (SValy > 0) { //forward
      digitalWrite(8, LOW);
      digitalWrite(12, LOW); //start moving
    } else if ((SValy < 0) && (GetWiredState() == 6))  { //backward
      digitalWrite(8, HIGH);
      digitalWrite(12, LOW); //start moving
    }  else {//stop
      digitalWrite(12, HIGH); //Stop moving
    }

  } else { // arm
    // clear stored values, if Joy stick has nonzero values
    if (SValx != 0 || SValy != 0) {
      XConnect = 0;
      YConnect = 0;
      FoundPos = false;
      HomePos = false;
    } else if ((SValx == 0) && (SValy == 0))
      if ((SValy <= 10) && (armAngle != 0))
        return;// to avoid misssending from Joy Stick

    // gliding avarage
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

    if ((SPWMy < 180) && (SPWMx < SERVOMIDDLE + 30) && (SPWMx > SERVOMIDDLE - 30))
      HomePos = true;

  }
}

void Smove() {
  //arm
  if (SPWMx > SPWMxold) {
    SPWMxold++;
    pwm.setPWM(0, 0, SPWMxold);

  } else if (SPWMx < SPWMxold) {
    SPWMxold--;
    pwm.setPWM(0, 0, SPWMxold);

  }
  if (SPWMy > SPWMyold) {
    SPWMyold++;
    pwm.setPWM(4, 0, SPWMyold);

    if (SPWMy - SPWMyold < 20) {
      if (SPWMy - SPWMyold < 10) {
        SmoveMinDelay = SmoveDefaultDelay + 7;
      } else
        SmoveMinDelay = SmoveDefaultDelay + 3;
    }
    else
      SmoveMinDelay = SmoveDefaultDelay;

  } else if (SPWMy < SPWMyold) {
    SPWMyold--;
    pwm.setPWM(4, 0, SPWMyold);

    if (SPWMyold - SPWMy < 20) {
      if (SPWMyold - SPWMy < 10) {
        SmoveMinDelay = SmoveDefaultDelay + 7;
      } else
        SmoveMinDelay = SmoveDefaultDelay + 3;
    }
    else
      SmoveMinDelay = SmoveDefaultDelay;
  }

  //driving  direction
  if (SPWMDirect > SPWMDirectold ) {
    SmoveMinDelay = 2;
    SPWMDirectold ++;
    pwm.setPWM(8, 0, SPWMDirectold);

  } else if (SPWMDirect < SPWMDirectold) {
    SmoveMinDelay = 2; 
    SPWMDirectold --;
    pwm.setPWM(8, 0, SPWMDirectold);
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
