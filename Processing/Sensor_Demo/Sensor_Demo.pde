import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

Serial myPort;

String data="";

int lm;
final int buffReadUntil = 17;

float roll, pitch, yaw;
Pose CubePose = new Pose(0,0,0,0,0,0);


void settings(){
  size (800, 800, P3D);
  myPort = new Serial(this, "COM5", 38400);
  delay(1000);
  

  myPort.buffer(buffReadUntil);
}
Plane pl = new Plane(1,1,1);
void draw() {
  showAxis();
  pushMatrix();
  showAxis();
  translate(width/2, height/2, 0);
  background(233);
  text("Roll: " + int(CubePose.Rx) + "     Pitch: " + int(CubePose.Ry), -100, 265);
  
  //translate(CubePose.X,CubePose.Y,CubePose.Z);
  rotateX(radians(CubePose.Rx));
  rotateZ(radians(CubePose.Ry));
  rotateY(radians(CubePose.Rz));
  lights();
  // 3D 0bject
  textSize(30);  
  fill(0, 76, 153);
  scale(90);
  
  pl.display();
 
  popMatrix();
}
// Read data from the Serial Port
void showAxis(){
  float len = 40;
  stroke(255, 0, 0);
  line(0, 0, 0, len, 0, 0);
  stroke(0, 255, 0);
  line(0, 0, 0, 0, len, 0);
  stroke(0, 0, 255);
  line(0, 0, 0, 0, 0, len);
  noStroke();
}
