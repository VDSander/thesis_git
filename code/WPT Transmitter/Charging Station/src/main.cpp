#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial uartSerial(5,4);

const int aantalPotValues = 14;
byte potValues [aantalPotValues] = {127,126,125,124,123,122,121,120,119,118,117,116,115,114};
byte potVal;
int pwmValue = 150;
int outputEnable = 7;
int pwmPinVCO = 6;
int index = -1;
int lastIndex = index;

int ledPin = 8;

void potReg(byte potVal);
int handleMessage(float val, int index);

void setup() {
  Serial.begin(9600);
  Serial.println("Laadstation OFF");
  uartSerial.begin(4800);

  Wire.begin();
  
  pinMode(outputEnable, OUTPUT);
  pinMode(pwmPinVCO, OUTPUT);
  analogWrite(pwmPinVCO, pwmValue);

  digitalWrite(outputEnable, LOW);
  potVal = potValues[0]; potReg(potVal);

  lastIndex = index;
}

void loop() {
  digitalWrite(ledPin, digitalRead(outputEnable));
  float val = uartSerial.parseFloat();
  if (uartSerial.read() == '\n'){
    if (val == -1 || val == 0 || val == 1 || val == 2){
      Serial.print("\nmessage received: "); Serial.println(val);
      index = handleMessage(val, index);
      uartSerial.print(val); uartSerial.print("\n");    }
    if (lastIndex != index){
      if (index == -1) {
        digitalWrite(outputEnable, LOW);
        potVal = potValues[0]; potReg(potVal);}
      else {
        potVal = potValues[index]; potReg(potVal);
        digitalWrite(outputEnable, HIGH);}
      lastIndex = index;
    }
  }
}

void potReg(byte pot){
  Wire.beginTransmission(46);
  Wire.write(byte(0x00));
  Wire.write(pot);
  Wire.endTransmission();
}

int handleMessage(float val, int index){
  if ((val == 1 && index == aantalPotValues-1) || (val == 2 && index == 0)){
    float errorCode = 404;
    uartSerial.print(errorCode); uartSerial.print("\n");  }
  else if (val == -1 && index >= 0){
    Serial.println("Stop handled");
    index = -1;}
  else if (val == 0 && index == -1){
    Serial.println("Start handled");
    index = 0;}
  else if (val == 1){
    Serial.println("More handled");
    index++;}
  else if (val == 2){
    Serial.println("Less handled");
    index--;}
  return index;
}