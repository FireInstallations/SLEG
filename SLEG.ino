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
unsigned long DriveLastJoysticktime; // Time since last joystick value while driving

byte StablelizerLastWireState; // if the last state was the same as the state now the last correction failed
bool StablelizerTryReconnect; //if the stablizer should activate
byte StablelizerCounter; //counts how many failed attemts of correcting the arm where made

/*
   JoysitckMode describes if the joystick moves the whole bus or the arm.
   true means the bus
   false means the arm
*/
bool JoystickMode = false;

// makes every secound an beeping sound if the accu is to low
bool ShouldLowAccuBeep = false;

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

  StablelizerTryReconnect = false;
  StablelizerLastWireState = 111;
  StablelizerCounter = 0;

  pinMode(4, INPUT); // High if BT connected
  BTSerial.begin(115200);  // HC-05 speed set in AT command mode
  printDelay = millis(); // timer for printing
  SmoveDelayTime = millis(); //time between Servo actions
  DriveLastJoysticktime = millis(); //time until app signal is assumed to be lost

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

  } else  if ((IsDriving) && ( millis() - DriveLastJoysticktime > 300)) {
    //sometimes the bluetooth connection looses enough values so the bus doens't stop
    //stop the bus
    HandleJoystick (0, 0);
  }

  if (millis() - printDelay > 1000) {

    Serial.println("wired detected    " + (String)analogRead(A6));
    Serial.println("Accu voltage    " + (String)analogRead(A7));
    Serial.println("Wired State    " + (String)GetWiredState()  );
    Serial.println("joystick Y: " + (String)SPWMy  );
    Serial.println("last connect coordinates " + (String)XConnect + "; " + (String)YConnect);
    Serial.println("HomeProgress   " + (String)HomeProgress);
    Serial.println("JoystickMode: " + (String)JoystickMode);
    Serial.println("IsDriving: " + (String)IsDriving);
    Serial.println("SPWMxold: " + (String)SPWMxold);

    Serial.println("WireState: " + (String)GetWiredState());

    Serial.println();
    printDelay = millis();

    if ((analogRead(A7) < 950) && (getLastDiget(GetWiredState()) == 0)) {
      if (ShouldLowAccuBeep)
        digitalWrite(13, LOW);
      else
        digitalWrite(13, HIGH);

      ShouldLowAccuBeep = !ShouldLowAccuBeep;

    } else if (!ShouldLowAccuBeep) {
      digitalWrite(13, HIGH);
      ShouldLowAccuBeep = false;
    }
  }

  AutoWiring();

  // store values if properly connected and disable joystick
  if (getLastDiget(GetWiredState()) > 1) {
    XConnect = SPWMxold;
    YConnect = SPWMyold;
    FoundPos = true;
    if (!JoystickMode) // only disable the joystick if its currintly controling the arm.
      JoyStickEnable = false;

    StablelizerTryReconnect = true;
  }

  if (millis() - SmoveDelayTime >= SmoveMinDelay) {
    if ((JoystickMode) && (!digitalRead(4))) { // if we are potentially moving but loosing BT connection, just stop the bus.
      IsDriving = false; // set the state to currently standing.
      analogWrite(3, 0); // set velocity to 0.
      digitalWrite(12, HIGH); //Stop moving

      Serial.println("Awww man, I lost control again! :/");
    }

    if (StablelizerTryReconnect && (IsDriving)) {
      ArmStablelizer ();
    }

    Smove();
    SmoveDelayTime = millis();
  }

  if (HomeProgress > 0)Homing();
}

void Homing() {
  switch (HomeProgress) {

    case 1: // start sequence move y
      StablelizerTryReconnect = false;

      if (!JoystickMode) { // disable all BT-input if Joystick is controling the arm
        BTEnable = false;
      } else {
        ButtonEnable = false; // disable just button input if we are currintly driving so we don't lock the bus movement
      }

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
        if (!JoystickMode) { // enable all BT-input if Joystick is controling the arm
          BTEnable = true;
          Serial.flush(); // clear all the old values.
        } else {
          ButtonEnable = true; // enable just button input if we are currintly driving
        }
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
      if (getLastDiget(GetWiredState()) == 0) //if not connected
        HomeProgress = 1; // return home
      break;
  }
}

byte GetWiredState () {
  /*  returns a 3 diget number, which is build of the x and y states and a state indiffer (xys).
      x relates to the state of the arm in horicontal direction, y in vertical direction,
      s is an shortcut if you just want to know if we are properly connecet or not at all.
      these 2 states are importend to know at different times in the program.
      so 231 corresponds to x == 2, y == 3 and s == 1.
      the states start with 0 and going by integer increments up to 2 for x and s, while for y up to 3.

      plan x/y    | x/y written out | s for same x/y values
      ------------|-----------------|----------------------
          y       |                 |
          3       |       03 13 23  |  0 0 0
          2       | ^     02 12 22  |  1 2 1
      x 0 1 2 x   | |     01 11 21  |  1 1 1
          0       | y     00 10 20  |  0 0 0
          y       |   x ->          |
                 <=>               ==>


      x == 0 means the arm is too far left, while x == 2 means it is to far right.
      y == 2 and y == 3 mean the arm is too far up.
      these 4 possible states are determined by the limit switches of the arm.
      the secound information in the return value is if  the arm is connected to the power line.
      if it is but the limit switch  is turned on, y has a value of 2,
      if the switch is on but the mesured voltage is to low it goes up to 3.
      this usually means we missed the right spot for the arm o and we are far to high, stuck at
      the left or right powerline.
      if no limit switch is on and we have a high enough volate the return number will be 11 (x == 1 & y == 1),
      this is the perfect state an we allways want to reach it.
      y == 0 means the y switch didn't trigger and the voltage doesn't fit. It could indicate we are just a bit too
      low, but we don't now for sure. There is no simple way of nowing if there is even a line at this point
      and therefor no simple way of resolving it. In the stabilizer subrutine we will try to move up a bit,
      but if this fails we will return home, similar to case y == 3.
      s is 0 if the voltage is to low, so no real connection was found (y == 0 or y == 3).
      if we are at a good spot to return to (connectet and upper limit switch is on since the automatic rewirering
      mostly doesn't reach as high) s will have the value of 2.
      if we are technically at the sweet spot for the arm or just a bit of s will have the default value of 1.

      x y | s | translates to                                                         | means                                                 | recommendation of resolvage
      ----|-----------------------------------------------------------------------|-------------------------------------------------------|------------------------------------------------------------------------------
      0 0 | 0 | left limit switch triggert; no votage; upper good                     | too low and too left or left of both power lines      | move gently up and right, if it not helps return home
      0 1 | 1 | left limit switch triggert; voltage good; upper good                  | good spot, just a bit too left                        | move right
      0 2 | 1 | left limit switch triggert; voltage good; upper switch  triggert      | good spot, just too left and high                     | move right and a bit down
      0 3 | 0 | left limit switch triggert; no voltage; upper limit switch  triggert  | missed the lines, now too high and left               | return home
      1 0 | 0 | x good; no voltage; upper switch goood                                | we have no idea where we are, since nothing indicates | try your luck and move up, if it not helps sacrifice a newborn or return home
      1 1 | 1 | x good; voltage good; upper goood (=> y good)                         | perfect spot try to stay here!                        | touch nothing. No! not even this. just stop!
      1 2 | 2 | x good; voltage good; upper switch  triggert                          | good spot, just too high                              | move a bit down
      1 3 | 0 | x good; no voltage; upper switch triggert                             | missed the lines, now too high                        | return home
      2 0 | 0 | right limit switch triggert; no votage; upper goood                   | too low and too right or right of both power lines    | move gently up and left, if it not helps return home
      2 1 | 1 | right limit switch triggert; voltage good; upper goood                | good spot just too right                              | move left
      2 2 | 1 | right limit switch triggert; voltage good; upper switch  triggert     | good spot just too right and high                     | move left and a bit down
      2 3 | 0 | right limit switch triggert; no voltage; upper limit switch  triggert | missed the lines, now too high and right              | return home

      to make it simple: we can handle erverything as long as y (the secound diget) is 1 or 2.
      ---
      the following logic enables to control search for contact and driving with contact
  */

  int voltage = analogRead(A6);
  bool toohigh = !digitalRead(6); // note that low means, that the switch is actuated
  bool tooleft = !digitalRead(7);
  bool tooright = !digitalRead(5);

  byte result = 111; // lets think positiv and assume we are in the perfect spot.

  if (tooright) {
    result -= 100; // x == 0
  } else if (tooleft)
    result += 100; // x == 2
  // else x == 1; is default value

  if ((voltage > 650) || (IsDriving && (voltage > 725) )) {
    if (toohigh) {
      result += 10; //y == 2
      result += 1; // s == 2
    } //y == 1 and s == 1 are the default values
  } else if (toohigh) {
    result += 20; //y == 3
    result -= 1; // s == 0
  } else {
    result -= 10; //y == 0
    result -= 1; // s == 0
  }

  return result;
}

void ArmStablelizer () {
  /*
    trys to hold the arm at the wire if possible.
    for more information about GetWiredState-numbers please reffer to the dokumentaion of GetWiredState.
  */

  //wait for arm to stop moving
  if ((SPWMx != SPWMxold) || (SPWMy != SPWMyold) )
    return;

  //cach wirestate
  byte WiredStateNow = GetWiredState();

  // was the last correction successfull?
  if ((WiredStateNow == StablelizerLastWireState) && (WiredStateNow != 111)) {
    if (StablelizerCounter <= 100) //no but maby the next will?
      StablelizerCounter++;
    else {
      StablelizerLastWireState = 111; //last 100 correction where unsuccessfull.
      HomeProgress = 1; //return home
      StablelizerCounter = 0;

      if (!JoystickMode) //reenable joystick
        JoyStickEnable = true;
    }
  }

  switch (WiredStateNow) {
    case 0: // too low and too left or left of both power lines
      // --> move gently up and right, if it not helps return home
      SPWMx = constrain(SPWMx + (SERVOMAX - SERVOMIN) * 0.06, SERVOMIN, SERVOMAX);
      SPWMy = constrain(SPWMy + (SERVOMAX - SERVOMIN - 30) * 0.03, SERVOMIN + 30, SERVOMAX);
      break;
    case 11: // good spot, just a bit too left
      // move right
      SPWMx = constrain(SPWMx + (SERVOMAX - SERVOMIN) * 0.06, SERVOMIN, SERVOMAX);
      break;
    case 21: // good spot, just too left and high
      //--> move right and a bit down
      SPWMx = constrain(SPWMx + (SERVOMAX - SERVOMIN) * 0.06, SERVOMIN, SERVOMAX);
      SPWMy = constrain(SPWMy - (SERVOMAX - SERVOMIN - 30) * 0.03, SERVOMIN + 30, SERVOMAX);
      break;
    case 030: // missed the lines, now too high and left
    case 130: // missed the lines, now too high
    case 230: // missed the lines, now too high and right
      HomeProgress = 1; //return home
      break;
    case 100: // we have no idea where we are, since nothing indicates
      //try your luck and move up, if it not helps return home
      SPWMy = constrain(SPWMy + (SERVOMAX - SERVOMIN - 30) * 0.03, SERVOMIN + 30, SERVOMAX);
      break;
    case 111: // perfect spot try to stay here!
      break;
    case 122: // good spot, just too high
      //move a bit down
      SPWMy = constrain(SPWMy - (SERVOMAX - SERVOMIN - 30) * 0.03, SERVOMIN + 30, SERVOMAX);
      break;
    case 200: // too low and too right or right of both power lines
      //move gently up and left, if it not helps return home
      SPWMx = constrain(SPWMx - (SERVOMAX - SERVOMIN) * 0.06, SERVOMIN, SERVOMAX);
      SPWMy = constrain(SPWMy + (SERVOMAX - SERVOMIN - 30) * 0.03, SERVOMIN + 30, SERVOMAX);
      break;
    case 211: // good spot just too right
      //move left
      SPWMx = constrain(SPWMx - (SERVOMAX - SERVOMIN) * 0.06, SERVOMIN, SERVOMAX);
      break;
    case 221: // good spot just too right and high
      //move left and a bit down
      SPWMx = constrain(SPWMx - (SERVOMAX - SERVOMIN) * 0.06, SERVOMIN, SERVOMAX);
      SPWMy = constrain(SPWMy - (SERVOMAX - SERVOMIN - 30) * 0.03, SERVOMIN + 30, SERVOMAX);
      break;
    default:
      Serial.println("[Stabbilizer] How did we get here?");
      Serial.println(WiredStateNow);
      break;
  }

  StablelizerLastWireState = WiredStateNow;

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

        if (getLastDiget(GetWiredState()) == 0) { // if not connected but also not home return home
          if (!HomePos)
            HomeProgress = 1;
        }

        JoystickMode = true;
        JoyStickEnable = true;
      }
      break;

    case 3: //" ⃤ " Connect wire if position of wire known, not connected and also not moving
      if ((!IsDriving) && (FoundPos) && (getLastDiget(GetWiredState()) == 0) )
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
    //the joystick values reach from -100 to 100 but ousing the full range makes the bus hard to control
    SPWMDirect = map(SValx, 140, -140, SERVOMIN, SERVOMAX);

    //the bus would be able to drive faster --> 60 up to 255 but not only the bus gets very hard to control
    //but also the arm servo would be to slow to move aloung.
    int velocity = map(abs(SValy), 0, 100, 60, 110);
    analogWrite(3, velocity); // set velocity

    if (velocity > 60) {
      IsDriving = true;
      DriveLastJoysticktime = millis();
    }

    if (SValy > 0) { //forward
      digitalWrite(8, LOW);
      digitalWrite(12, LOW); //start moving
    } else if ((SValy < 0) && (GetWiredState() == 100))  { //backward if  arm is not attached
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

    if ((SPWMy < 180) && (SPWMx < SERVOMIDDLE + 30) && (SPWMx > SERVOMIDDLE - 30)) {
      HomePos = true;
      StablelizerTryReconnect = false;
    }

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
  if (SPWMy > SPWMyold) { //up
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

  } else if (SPWMy < SPWMyold) { //down
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
  for (char c : str) {

    if (!isDigit(c))
      return false;
  }
  return true;
}

byte getLastDiget (byte num) {
  return num % 10;
}
