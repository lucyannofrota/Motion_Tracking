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
  int charCount = 6;
  int len = bluetooth.available();
  if(len > 0){
    bluetooth.readBytes(&buf[0], 1);
    if(buf[0] == '{'){
      bluetooth.readBytes(&buf[1], 1);
      switch(buf[1]){
        case 'P':
          bluetooth.readBytes(&buf[2], charCount*2); // ler payload
          
          bluetooth.readBytes(&buf[charCount*2 + 2], 2); // ler caracteres terminadores
          Serial.write(buf,16);
          break; 
      }
    }
  }
  delay(25);
}
