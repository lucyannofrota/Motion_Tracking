#include <SoftwareSerial.h>

/*
 * Necessario utilizar '?' apos o comando
AT COMMANDS
AT : Ceck the connection.
AT+NAME : See default name
AT+ADDR : see default address
AT+VERSION : See version
AT+UART : See baudrate
AT+ROLE: See role of bt module(1=master/0=slave)
AT+RESET : Reset and exit AT mode
AT+ORGL : Restore factory settings
AT+PSWD: see default password
AT+BIND: Master Connect (Slave addr: xxxx,xxx,xx)
https://s3-sa-east-1.amazonaws.com/robocore-lojavirtual/709/HC-05_ATCommandSet.pdf
 * 
 */

/* 
 * BaudRate 38400
 * Name BT1, BT2
 * PSW 1234, 1234
 */
#include <stdio.h>
#include <string.h>

#define ENABLE_PIN 4


#define RX_PIN 3 // Voltage Divider 3.3V
#define TX_PIN 2

SoftwareSerial ATDevice(TX_PIN, RX_PIN); // RX, TX -> Inverted For Module

  String command(const char *toSend, unsigned long milliseconds) {
    String result;
    Serial.print("Sending: ");
    Serial.println(toSend);
    ATDevice.println(toSend);
    unsigned long startTime = millis();
    Serial.print("Received: ");
    while (millis() - startTime < milliseconds) {
      if (ATDevice.available()) {
        char c = ATDevice.read();
        Serial.write(c);
        result += c;  // append to the result string
      }
    }
  Serial.println();  // new line after timeout.
  return result;
}

String command(const char *toSend) {
    String result = command(toSend,500);
    delay(500);
    return result;
}

String Rdata;

void setup() {

  Serial.begin(38400);
  
//  pinMode(VCC_PIN,OUTPUT); digitalWrite(VCC_PIN,HIGH);
  pinMode(ENABLE_PIN,OUTPUT); digitalWrite(ENABLE_PIN,HIGH);
  
  Serial.println("Sending AT commands...");
  
  ATDevice.begin(38400);
  delay(300);
  command("AT");
  Rdata = command("AT+NAME?");
  if(Rdata.indexOf("BT1") != -1){
    command("AT+UART=38400,0,0");
    command("AT+ROLE=1");
    command("AT+RMAAD");
    command("AT+PSWD=1234");
    command("AT+PAIR=0020,10,000240");
    command("AT+BIND=0020,10,000240");
    command("AT+LINK=0020,10,000240");
    command("AT+CMODE=0");
    command("AT+RESET");
  }
  else{
    if(Rdata.indexOf("BT2") != -1){
      command("AT+UART=38400");
      command("AT+ROLE=0");
      command("AT+PSWD=1234");
      command("AT+ADDR?");
      command("AT+RESET");  
    }
    else{
      command("AT+ADDR?");
      command("AT+PSWD?");
      command("AT+RESET"); 
    }
  }
}






//void readBTCOM(void){
//  if (ATDevice.available()) Serial.write(ATDevice.read());
//}

void loop()

{
if (ATDevice.available())

Serial.write(ATDevice.read());

if (Serial.available())

ATDevice.write(Serial.read());

}
