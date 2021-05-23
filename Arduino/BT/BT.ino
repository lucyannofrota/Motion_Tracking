#include <SoftwareSerial.h>
#include <string.h>
#define ENABLE_PIN 7
#define BUFFER_LEN 64
const int pinoRX = 2;
const int pinoTX = 3;

char buf[BUFFER_LEN];

SoftwareSerial bluetooth(pinoRX, pinoTX);
 
void setup(){
  pinMode(ENABLE_PIN,OUTPUT); digitalWrite(ENABLE_PIN,LOW);
  Serial.begin(38400);
  bluetooth.begin(38400);
  delay(100);
}

void loop(){
  if (bluetooth.available()){
    readSerial();
  }
  if (Serial.available())
    bluetooth.write(Serial.read());
}

void readSerial(){
  memcpy(buf,0,BUFFER_LEN);
  int len = bluetooth.available();
  if(len > 0){
    bluetooth.readBytes(buf, 11);
    Serial.write(buf,11);
  }
  delay(25);
}
