import processing.serial.*;
import java.awt.event.KeyEvent;
import java.io.IOException;

String port = "COM5";
int baudrate = 38400;

import static javax.swing.JOptionPane.*;

import grafica.*;

Serial myPort;

public enum _TAB {
    GRAPHS, 
    BODY, 
    SENSOR
}

_TAB currentTAB = _TAB.GRAPHS;

PGraphics BackGround;

boolean SetupFlag = false;
boolean InitializationFlag = false;

void settings() {
  size(1920, 1080-45, P2D);
}

final int screenWidth = 1920, screenHeight = 1080;
float DrawWindowPos[];
float DrawWindowDim[];
void setup() {
  surface.setTitle("Motion Tracking");
  DrawWindowPos = new float[]{5, height*0.1+5};
  DrawWindowDim = new float[]{width-10, height*0.9-25};
  Graphs = new GraphsC(this, int(DrawWindowPos), int(DrawWindowDim));
  Body3D = new Body3DC(int(DrawWindowDim));
  Sensor3D = new Sensor3DC(int(DrawWindowDim));

  lm = millis();


  BackGround = createGraphics(screenWidth, screenHeight);
  // Gradient
  BackGround.beginDraw();

  BackGround.rectMode(CORNER);
  BackGround.fill(#EAEAEA);
  BackGround.rect(0, 0, 50, 50);
  
  for (int i = 0; i <= screenHeight; i++) {
    float inter = map(i, 0, screenHeight, 0, 1);
    color c = lerpColor(#495558, #4695A2, inter);
    BackGround.stroke(c);
    BackGround.line(0, i, screenWidth, i);
  }
  BackGround.endDraw();

  delay(7000);
  //final String port = showInputDialog("Please enter the COM Port: ");
  //if (port == null)   exit();
  //else{
  //  myPort = new Serial(this, port, 115200);
  //  SetupFlag = false;
  //}
  myPort = new Serial(this, port, baudrate);
  myPort.buffer(buffReadUntil);
  InitializationFlag = true;
}

void draw() {
  if(SetupFlag){
    return;
  }
  if(InitializationFlag == false) return;

  image(BackGround, 0, 0);
  switch(currentTAB) {
  case GRAPHS:
    Graphs.draw(5, height*0.1+5);
    break;
  case BODY:
    image(Body3D.draw(), 5, height*0.1+5);
    break;
  case SENSOR:
    image(Sensor3D.draw(), 5, height*0.1+5);
    break;
  }
}




















///////////////
//Mouse Events/
///////////////


void mouseClicked() {
  if((mouseX > DrawWindowPos[0] && mouseX < ((DrawWindowPos[0])+DrawWindowDim[0])) &&
     (mouseY > DrawWindowPos[1] && mouseY < ((DrawWindowPos[1])+DrawWindowDim[1]))
    ){
    switch(currentTAB){
      case GRAPHS:
        Graphs.mouseClicked();
      break;
      case BODY:
        Body3D.mouseClicked();
      break;
      case SENSOR:
        Sensor3D.mouseClicked();
      break;
    }
  }
}

void mouseDragged() {
  if((mouseX > DrawWindowPos[0] && mouseX < ((DrawWindowPos[0])+DrawWindowDim[0])) &&
     (mouseY > DrawWindowPos[1] && mouseY < ((DrawWindowPos[1])+DrawWindowDim[1]))
    ){
    switch(currentTAB){
      case GRAPHS:
        Graphs.mouseDragged();
      break;
      case BODY:
        Body3D.mouseDragged();
      break;
      case SENSOR:
        Sensor3D.mouseDragged();
      break;
    }
  }
}

void mouseWheel(MouseEvent event) {
  if((mouseX > DrawWindowPos[0] && mouseX < ((DrawWindowPos[0])+DrawWindowDim[0])) &&
     (mouseY > DrawWindowPos[1] && mouseY < ((DrawWindowPos[1])+DrawWindowDim[1]))
    ){
    switch(currentTAB){
      case GRAPHS:
        Graphs.mouseWheel(event);
      break;
      case BODY:
        Body3D.mouseWheel(event);
      break;
      case SENSOR:
        Sensor3D.mouseWheel(event);
      break;
    }
  }
}


////////////////////////
/// Keyboard Event /////
////////////////////////
void keyPressed() {
  if(key == ' '){
    switch(currentTAB){
      case GRAPHS:
      currentTAB = _TAB.SENSOR;
      break;
      case BODY:
      currentTAB = _TAB.GRAPHS;
      break;
      case SENSOR:
      currentTAB = _TAB.BODY;
      break;
    }
  }
  if(key == 'r' || key == 'R'){
    switch(currentTAB){
      case GRAPHS:
      //currentTAB = _TAB.SENSOR;
      break;
      case BODY:
        Body3D.keyPressed(key);
      //currentTAB = _TAB.GRAPHS;
      break;
      case SENSOR:
        Sensor3D.keyPressed(key);
      //currentTAB = _TAB.BODY;
      break;
    }
  }
}






///////////////
///Structures//
///////////////

class Pose {
  float X = 0, Y = 0, Z = 0;
  float Rx = 0, Ry = 0, Rz = 0;

  Pose() {
  }

  Pose(float x_, float y_, float z_, float rx_, float ry_, float rz_) {
    this.X = x_; 
    this.Y = y_; 
    this.Z = z_;
    this.Rx = rx_; 
    this.Ry = ry_; 
    this.Rz = rz_;
  }

  void setZero() {
    this.X = 0; 
    this.Y = 0; 
    this.Z = 0;
    this.Rx = 0; 
    this.Ry = 0; 
    this.Rz = 0;
  }

  void print() {
    println("Pose: \nT -> ("+this.X+','+this.Y+','+this.Z+")\nR -> ("+this.Rx+','+this.Ry+','+this.Rz+")\n");
  }
}

class IMU {
  public float ax, ay, az;
  public float rx, ry, rz;

  IMU() {
    ax = 0; 
    ay = 0; 
    az = 0;
    rx = 0; 
    ry = 0; 
    rz = 0;
  }

  IMU(float ax_, float ay_, float az_, float rx_, float ry_, float rz_) {
    ax = ax_; 
    ay = ay_; 
    az = az_;
    rx = rx_; 
    ry = ry_; 
    rz = rz_;
  }

  void def(float ax_, float ay_, float az_, float rx_, float ry_, float rz_) {
    ax = ax_; 
    ay = ay_; 
    az = az_;
    rx = rx_; 
    ry = ry_; 
    rz = rz_;
  }
}
