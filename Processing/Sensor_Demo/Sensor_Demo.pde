import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

Serial myPort;

String data="";

int lm;

float roll, pitch, yaw;
Pose CubePose = new Pose(0,0,0,0,0,0);

void setup() {
  size (800, 800, P3D);
  myPort = new Serial(this, "COM3", 38400);
  delay(1000);
  myPort.bufferUntil('\n');
  lm = millis();
}
void draw() {
  translate(width/2, height/2, 0);
  background(233);
  text("Roll: " + int(CubePose.Rx) + "     Pitch: " + int(CubePose.Ry), -100, 265);
  translate(CubePose.X,CubePose.Y,CubePose.Z);
  rotateX(radians(CubePose.Rx));
  rotateZ(radians(CubePose.Ry));
  rotateY(radians(CubePose.Rz));

  // 3D 0bject
  textSize(30);  
  fill(0, 76, 153);
  box (386, 40, 200); // Draw box
  textSize(25);
  fill(255, 255, 255);
}
// Read data from the Serial Port

void rd(){
  
}

byte[] COMBuff = new byte[64];

void serialEvent (Serial myPort) { 
  int ac = millis(); 
  println("Time:" + (ac-lm));
  lm = ac;
  int charCount = 6;
  offset of = new offset(0);
  if (myPort.available()>0) {
    byte c = byte(myPort.read());
    if(c == '{'){
      c = byte(myPort.read());
      switch(c){
        case 'P':
          COMBuff = myPort.readBytes(charCount * 2);
          readPose(COMBuff,of,CubePose); 
          break;
      }
    }
  }
}
