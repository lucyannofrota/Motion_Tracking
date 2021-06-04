import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;
import grafica.*;

Serial myPort;
int a;
String data="";

int lm;

final int buffReadUntil = 17;

float roll, pitch, yaw;
Pose CubePose = new Pose(0,0,0,0,0,0);

camera_Obj cam;

body bd1 = new body(170); 

myGraphWindow PG = new myGraphWindow();

void settings(){
  size (800/800, 800/800, P3D);
  myPort = new Serial(this, "COM15", 38400);
  delay(1000);
  myPort.buffer(buffReadUntil);
}

void setup() {
  surface.setTitle("Main Window");


  lm = millis();
  
  cam = new camera_Obj(float(0),3*float(170)/4,float(100));
  cam.update();
  
 
}
//37

void draw() {
  background(#DCDCDC);
  float len = 10000;
  stroke(255, 0, 0);
  line(0, 0, 0, len, 0, 0);
  stroke(0, 255, 0);
  line(0, 0, 0, 0, len, 0);
  stroke(0, 0, 255);
  line(0, 0, 0, 0, 0, len);
  noStroke();
  draw_floor();
  bd1.draw();
  
  
  
  
  //box(10,20,30);
  //sphere(50);
  //translate(width/2, height/2, 0);
  //background(233);
  //text("Roll: " + int(CubePose.Rx) + "     Pitch: " + int(CubePose.Ry), -100, 265);
  //translate(CubePose.X,CubePose.Y,CubePose.Z);
  //rotateX(radians(CubePose.Rx));
  //rotateZ(radians(CubePose.Ry));
  //rotateY(radians(CubePose.Rz));

  //// 3D 0bject
  //textSize(30);  
  //fill(0, 76, 153);
  //box (386, 40, 200); // Draw box
  //textSize(25);
  //fill(255, 255, 255);
  
}
