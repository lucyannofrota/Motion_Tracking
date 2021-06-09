#include <SoftwareSerial.h>
#include <string.h>
#define ENABLE_PIN 7
#define BUFFER_LEN 64
const int pinoRX = 2;
const int pinoTX = 3;

char buf[BUFFER_LEN];
int ln;
SoftwareSerial bluetooth(pinoRX, pinoTX);
 
void setup(){
  pinMode(ENABLE_PIN,OUTPUT); digitalWrite(ENABLE_PIN,LOW);
  Serial.begin(38400);
  bluetooth.begin(38400);
  delay(100);
//  ln = millis();
}

void loop(){
  if (bluetooth.available()){
    readSerial();
  }
  if (Serial.available())
    bluetooth.write(Serial.read());
  
}

const int maxbuf = 17;
void readSerial(){
  memcpy(buf,0,BUFFER_LEN);
btread:
  int len = bluetooth.available();
  if(len > 0){
    if(len < maxbuf) delay(15);
    bluetooth.readBytes(&buf[0], 1);
    if(buf[0] == '{'){
      bluetooth.readBytes(&buf[1], 1);
      switch(buf[1]){
        case 'P':
          bluetooth.readBytes(&buf[2], 1+6*2 + 2); // ler payload (14)
          
          Serial.write(buf,17);
          
          break; 
        case 'S':
          bluetooth.readBytes(&buf[2], 1+6*2 + 2); // ler payload (14)
          
          Serial.write(buf,17);
          
          break; 
      }
      if(bluetooth.available() > maxbuf) goto btread;
      else return;
    }
    Serial.write(buf,1);
  }
  
}
