
//PGraphics Boby3DFB;
Body3DC Body3D;

class Body3DC {

  int DrawSize[];

  PGraphics IBuf;
  body b;
  
  camera_Obj cam;

  Body3DC(int [] Dsize) {
    DrawSize = Dsize;
    IBuf = createGraphics(DrawSize[0], DrawSize[1], P3D);
    b = new body(170, IBuf);
    cam = new camera_Obj(IBuf,float(0),3*float(170)/4,float(100));
    cam.update();
  }

  PGraphics draw() {
    
    final float len = 10000;
    cam.update();
    IBuf.beginDraw();
    IBuf.pushMatrix();
    //IBuf.rotateY(radians(180));
    IBuf.lights();
    IBuf.background(#676767);
    //IBuf.translate(0,0,-10);
    draw_floor(IBuf);
    IBuf.stroke(255, 0, 0);
    IBuf.line(0, 0, 0, len, 0, 0);
    IBuf.stroke(0, 255, 0);
    IBuf.line(0, 0, 0, 0, len, 0);
    IBuf.stroke(0, 0, 255);
    IBuf.line(0, 0, 0, 0, 0, -len);
    IBuf.noStroke();
    
    
    //IBuf.fill(#F70A0A);
    //IBuf.rotateX(radians(180));
    //IBuf.translate(width/2, -height/4, -200);
    //IBuf.scale(5);

    b.draw();
    IBuf.popMatrix();
    IBuf.endDraw();
    return IBuf;
  }

  void mouseClicked() {
    //println("Click");
  }


  void mouseWheel(MouseEvent e) {
    //println("Wheel");
    cam.fov+=float(e.getCount())/10;
    //println(cam.fov);
    if (cam.fov < 1) {
      cam.fov = 1;
    }
    if (cam.fov > 2) {
      cam.fov = 2;
    }
    cam.update();
  } 

  void mouseDragged() {
    //println("Drag");
    float dXm = mouseX-pmouseX, dYm = mouseY-pmouseY;
    switch(mouseButton) {
    case LEFT:
      cam.transl(dXm*0.9, dYm*0.9);
      break;
    case RIGHT:
      cam.rot(dXm, dYm);
      break;
    }
  }
}

void draw_floor(PGraphics IBuf) {
  IBuf.fill(#D59DDE);
  IBuf.box(10000, 0, 10000);
}

void drawCylinder(PGraphics IBuf ,int sides, float r1, float r2, float h) {
  float angle = 360 / sides;
  float halfHeight = h / 2;
  // top
  IBuf.beginShape();
  for (int i = 0; i < sides; i++) {
    float x = cos( radians( i * angle ) ) * r1;
    float y = sin( radians( i * angle ) ) * r1;
    IBuf.vertex( x, y, -halfHeight);
  }
  IBuf.endShape(CLOSE);
  // bottom
  IBuf.beginShape();
  for (int i = 0; i < sides; i++) {
    float x = cos( radians( i * angle ) ) * r2;
    float y = sin( radians( i * angle ) ) * r2;
    IBuf.vertex( x, y, halfHeight);
  }
  IBuf.endShape(CLOSE);
  // draw body
  IBuf.beginShape(TRIANGLE_STRIP);
  for (int i = 0; i < sides + 1; i++) {
    float x1 = cos( radians( i * angle ) ) * r1;
    float y1 = sin( radians( i * angle ) ) * r1;
    float x2 = cos( radians( i * angle ) ) * r2;
    float y2 = sin( radians( i * angle ) ) * r2;
    IBuf.vertex( x1, y1, -halfHeight);
    IBuf.vertex( x2, y2, halfHeight);
  }
  IBuf.endShape(CLOSE);
}

class body {

  float b_height;

  float b_head_r;
  float b_neck;
  float b_shoulders_hip_r;
  float b_chest;

  protected final PGraphics ib;
  void showAxis() {
    float len = 40;
    ib.stroke(255, 0, 0);
    ib.line(0, 0, 0, len, 0, 0);
    ib.stroke(0, 255, 0);
    ib.line(0, 0, 0, 0, len, 0);
    ib.stroke(0, 0, 255);
    ib.line(0, 0, 0, 0, 0, -len);
    ib.noStroke();
  }

  body(float _height, PGraphics p) {
    b_height = _height;
    b_head_r = 25/2;
    b_neck = 7.5;
    b_shoulders_hip_r = 28/2;
    b_chest = 45+2*b_shoulders_hip_r;
    ib = p;
  }

  void draw_head(float pos) {
    ib.pushMatrix();
    ib.translate(0, pos, 0);
    ib.fill(#FFD27E);
    ib.sphere(b_head_r);
    ib.popMatrix();
  }

  void draw_neck(float pos) {
    ib.pushMatrix();
    ib.translate(0, pos, 0);
    ib.fill(#FFD27E);
    ib.rotateX(PI/2);
    drawCylinder(ib,36, 7/2, 7.5/2, b_neck);
    ib.popMatrix();
  }

  void draw_shoulders(float pos) {
    ib.pushMatrix();
    ib.translate(0, pos, 0);
    ib.fill(#ff449f);
    ib.sphere(b_shoulders_hip_r-0.1);
    ib.popMatrix();
  }

  void draw_left_arm(float x, float y, float z, int showAxis) {
    ib.pushMatrix();
    ib.fill(#005f99);
    ib.rotateY(PI);
    ib.translate(-x, y, -z);
    float radius = b_shoulders_hip_r/2;
    ib.sphere(radius);
    if (showAxis == 1) {
      showAxis();
    }
    ib.pushMatrix();
    ib.rotateY(PI/2);
    ib.fill(#FFD27E);
    ib.translate(0,0,20+radius/2);
    drawCylinder(ib,36,3,3,40);
    ib.popMatrix();
    ib.popMatrix();
  }

  void draw_right_arm(float x, float y, float z, int showAxis) {
    ib.pushMatrix();
    ib.fill(#005f99);
    //rotateZ(PI/2);
    ib.translate(x, y, z);
    float radius = b_shoulders_hip_r/2;
    ib.sphere(radius);
    if (showAxis == 1) {
      showAxis();
    }
    ib.pushMatrix();
    ib.rotateY(PI/2);
    ib.translate(0,0,20+radius/2);
    ib.fill(#FFD27E);
    drawCylinder(ib,36,3,3,40);
    ib.popMatrix();
    ib.popMatrix();
  }

  void draw_chest(float pos, int showAxis) {
    ib.pushMatrix();
    ib.translate(0, pos, 0);
    ib.fill(#ff449f);
    ib.rotateX(PI/2);
    drawCylinder(ib,36, b_shoulders_hip_r, b_shoulders_hip_r, b_chest);
    if (showAxis == 1) {
      showAxis();
    }
    ib.popMatrix();
  }

  void draw_hip(float pos) {
    ib.pushMatrix();
    ib.translate(0, pos/32, 0);
    ib.fill(#005f99);
    ib.sphere(90);
    ib.popMatrix();
  }

  void draw() {
    ib.noStroke();
    //ib.lights();
    draw_head(b_height-b_head_r);
    draw_neck(b_height-2*b_head_r-b_neck/2+b_neck/4);
    draw_shoulders(b_height-2*b_head_r-b_neck+b_neck/3-b_shoulders_hip_r);
    draw_left_arm(-b_shoulders_hip_r, b_height-2*b_head_r-b_neck+b_neck/3-b_shoulders_hip_r+2, 0, 1);
    draw_right_arm(b_shoulders_hip_r, b_height-2*b_head_r-b_neck+b_neck/3-b_shoulders_hip_r+2, 0, 1);
    draw_chest(b_height-2*b_head_r-b_neck+b_neck/3-b_shoulders_hip_r-b_chest/2, 1);
    draw_hip(b_height-2*b_head_r-b_neck+b_neck/3-b_shoulders_hip_r-2*b_chest/2);
  }
  
  class joint{
    PlotGroupIMU sens;
    PGraphics ibuf;
    float []transform = {-1,0,0,0,
                          0,1,0,0,
                          0,0,-1,0};
    float [] rpy;
    joint(PGraphics buf, PlotGroupIMU s){
      ibuf = buf;
    }
    
    joint(PGraphics buf, PlotGroupIMU s, float [] transf){
      transform = transf;
      ibuf = buf;
    }
    
    void pushMatrix(){
      ibuf.pushMatrix();
      applyMatrix(
        transform[0],transform[1],transform[2],transform[3],
        transform[4],transform[5],transform[6],transform[7],
        transform[8],transform[9],transform[10],transform[11],
        transform[12],transform[13],transform[14],transform[15]
        );
      rpy = sens.getRPY();
      rotateX(rpy[0]);
      rotateY(rpy[1]);
      rotateZ(rpy[2]);
    }
    
    void popMatrix(){
      ibuf.popMatrix();
    }
  }
}






class camera_Obj {
  float x; 
  float y; 
  float z;
  float pitch; 
  float yaw;
  float fov; 
  
  final PGraphics parent;

  camera_Obj(PGraphics par,float x_, float y_, float z_) {
    parent = par;
    this.x = x_;
    this.y = y_;
    this.z = z_;
    this.pitch = 0;
    this.yaw = radians(-90);
    this.fov = 1.5;
  }

  void transl(float dXm, float dYm) {
    x+=dXm/3;
    y+=dYm/3;
    this.update();
  }

  void rot(float dXm, float dYm) {
    yaw-=dXm/500;
    if (yaw > 89) {
      yaw = 89;
    }
    if (yaw < -89) {
      yaw = -89;
    }
    pitch-=dYm/500;
    if (pitch > 89) {
      pitch = 89;
    }
    if (pitch < -89) {
      pitch = -89;
    }

    this.update();
  }

  void update() {
    parent.camera(x, y, z, x+cos(yaw)*cos(pitch), y+sin(pitch), z+sin(yaw)*cos(pitch), 0, -1, 0);
    parent.perspective(fov, float(width)/float(height), 0.001, 10000);
  }
}
//stroke(255, 0, 0);
//line(-1000, 0, 0, 1000, 0, 0);
//stroke(0, 255, 0);
//line(0, -1000, 0, 0, 1000, 0);
//stroke(0, 0, 255);
//line(0, 0, -1000, 0, 0, 1000);
