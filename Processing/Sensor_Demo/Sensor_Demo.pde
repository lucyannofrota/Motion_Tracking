import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

Serial myPort;

String data="";

float roll, pitch, yaw;
Pose CubePose = new Pose(0,0,0,0,0,0);
void setup() {
  size (800, 800, P3D);
  myPort = new Serial(this, "COM9", 38400);
  myPort.bufferUntil('\n');
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

void serialEvent (Serial myPort) { 
  data = myPort.readStringUntil('\n');
  if (data != null) {
    for(int i = 0; i < 6; i++){
      println(data.charAt(i));
    }
    CubePose.X = data.charAt(0)-48; CubePose.Y = data.charAt(1)-48; CubePose.Z = data.charAt(2)-48;
    CubePose.Rx = data.charAt(3)-48; CubePose.Ry = data.charAt(4)-48; CubePose.Rz = data.charAt(5)-48;
    CubePose.print();
    println(data);
  }
}
