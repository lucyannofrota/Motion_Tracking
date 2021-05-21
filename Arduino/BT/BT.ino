#include <SoftwareSerial.h>

#define ENABLE_PIN 7
 
const int pinoRX = 2;
const int pinoTX = 3;
 
SoftwareSerial bluetooth(pinoRX, pinoTX);
 
void setup(){
  pinMode(ENABLE_PIN,OUTPUT); digitalWrite(ENABLE_PIN,LOW);
  Serial.begin(38400);
  bluetooth.begin(38400);
  delay(100);
}

void loop(){
  if (bluetooth.available())
    Serial.write(bluetooth.read());
 
  if (Serial.available())
    bluetooth.write(Serial.read());
}
