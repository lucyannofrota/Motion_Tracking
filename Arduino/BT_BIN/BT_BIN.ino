#include <SoftwareSerial.h>

#define ENABLE_PIN 7
 
const int pinoRX = 2;
const int pinoTX = 3;
 
SoftwareSerial bluetooth(pinoRX, pinoTX);

int lt;
 
void setup(){
  pinMode(ENABLE_PIN,OUTPUT); digitalWrite(ENABLE_PIN,LOW);
  Serial.begin(38400);
  bluetooth.begin(38400);
  delay(100);
  lt = millis();
}

char buff[64];

//int ac;

size_t n;

void loop(){
  if (bluetooth.available()){
    n = bluetooth.readBytesUntil('\0', buff, 64);
    Serial.write(buff,n);
    Serial.write('\n');
//    ac = millis();
//    Serial.print(", Time: ");
//    Serial.println(ac-lt);
//    lt = ac;
    delay(25);
  }
 
  if (Serial.available()) bluetooth.write(Serial.read());

}
