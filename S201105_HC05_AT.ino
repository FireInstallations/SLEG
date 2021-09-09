/* It is necessary to power EN to 5V and after that VCC disconnect and connect again
 *  the diode should blink with 0.5 Hz
 * AT // should answer ok
 * AT+NAME=SLEG // set device name
AT+ROLE=0 // set to slave mode
AT+PSWD=0000 // set password
AT+UART=9600,0,0 // set serial connection to 9600 Baud
 most of the setting can be checked by for example AT+NAME
 */




#include <SoftwareSerial.h>

SoftwareSerial BTSerial(10, 11); // CONNECT BT RX PIN TO ARDUINO 11 PIN | CONNECT BT TX PIN TO ARDUINO 10 PIN

void setup() 
{
  pinMode(9, OUTPUT);  // this pin will pull the HC-05 pin 34 (key pin) HIGH to switch module to AT mode
  digitalWrite(9, HIGH); 
  Serial.begin(9600);
  Serial.println("Enter AT commands:");
  BTSerial.begin(38400);  // HC-05 default speed in AT command more
}

void loop()
{

  // Keep reading from HC-05 and send to Arduino Serial Monitor
  if (BTSerial.available())
    Serial.write(BTSerial.read());

  // Keep reading from Arduino Serial Monitor and send to HC-05
  if (Serial.available())
    BTSerial.write(Serial.read());
}
