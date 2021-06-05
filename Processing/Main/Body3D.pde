
//PGraphics Boby3DFB;
Body3DC Body3D;

class Body3DC{
  
  int DrawSize[];
  
  PGraphics IBuf;
  
  Body3DC(int [] Dsize){
    DrawSize = Dsize;
    IBuf = createGraphics(DrawSize[0],DrawSize[1],P3D);
  }
  
  PGraphics draw(){
    //cam.update();
    IBuf.beginDraw();
    IBuf.background(#676767);
    IBuf.fill(#F70A0A);
    IBuf.translate(width/2,height/2);
    IBuf.box(70);
    IBuf.lights();
    IBuf.endDraw();
    return IBuf;
  }
}



void draw_floor(){
  fill(#D59DDE);
  box(10000,0,10000);
}

void drawCylinder( int sides, float r1, float r2, float h)
{
    float angle = 360 / sides;
    float halfHeight = h / 2;
    // top
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r1;
        float y = sin( radians( i * angle ) ) * r1;
        vertex( x, y, -halfHeight);
    }
    endShape(CLOSE);
    // bottom
    beginShape();
    for (int i = 0; i < sides; i++) {
        float x = cos( radians( i * angle ) ) * r2;
        float y = sin( radians( i * angle ) ) * r2;
        vertex( x, y, halfHeight);
    }
    endShape(CLOSE);
    // draw body
    beginShape(TRIANGLE_STRIP);
    for (int i = 0; i < sides + 1; i++) {
        float x1 = cos( radians( i * angle ) ) * r1;
        float y1 = sin( radians( i * angle ) ) * r1;
        float x2 = cos( radians( i * angle ) ) * r2;
        float y2 = sin( radians( i * angle ) ) * r2;
        vertex( x1, y1, -halfHeight);
        vertex( x2, y2, halfHeight);
    }
    endShape(CLOSE);
}

class body{
  
  float b_height;
  
  float b_head_r;
  float b_neck;
  float b_shoulders_hip_r;
  float b_chest;
  
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
  
  body(float _height){
    b_height = _height;
    b_head_r = 14.8/2;
    b_neck = 7.5;
    b_shoulders_hip_r = 28/2;
    b_chest = 45+2*b_shoulders_hip_r;
  }
  
  void draw_head(float pos){
    pushMatrix();
    translate(0,pos,0);
    fill(#C84646);
    sphere(b_head_r);
    popMatrix();
  }
  
  void draw_neck(float pos){
    pushMatrix();
    translate(0,pos,0);
    fill(#C84646);
    rotateX(PI/2);
    drawCylinder(36,7/2,7.5/2,b_neck);
    popMatrix();
  }
  
  void draw_shoulders(float pos){
    pushMatrix();
    translate(0,pos,0);
    fill(#C84646);
    sphere(b_shoulders_hip_r);
    popMatrix();
  }
  
  void draw_left_arm(float x,float y,float z,int showAxis){
    pushMatrix();
    fill(#18B9C0);
    rotateY(PI);
    translate(-x,y,-z);
    sphere(b_shoulders_hip_r/2);
    if(showAxis == 1){
      showAxis();
    }
    popMatrix();
  }
  
  void draw_right_arm(float x,float y,float z,int showAxis){
    pushMatrix();
    fill(#18B9C0);
    //rotateZ(PI/2);
    translate(x,y,z);
    sphere(b_shoulders_hip_r/2);
    if(showAxis == 1){
      showAxis();
    }
    popMatrix();
  }
  
  void draw_chest(float pos,int showAxis){
    pushMatrix();
    translate(0,pos,0);
    fill(#C84646);
    rotateX(PI/2);
    drawCylinder(36,b_shoulders_hip_r,b_shoulders_hip_r,b_chest);
    if(showAxis == 1){
      showAxis();
    }
    popMatrix();
  }
  
  void draw_hip(float pos){
    pushMatrix();
    translate(0,pos,0);
    fill(#C84646);
    sphere(b_shoulders_hip_r);
    popMatrix();
  }
  
  void draw(){
    noStroke();
    lights();
    draw_head(b_height-b_head_r);
    draw_neck(b_height-2*b_head_r-b_neck/2+b_neck/4);
    draw_shoulders(b_height-2*b_head_r-b_neck+b_neck/3-b_shoulders_hip_r);
    draw_left_arm(-b_shoulders_hip_r,b_height-2*b_head_r-b_neck+b_neck/3-b_shoulders_hip_r,0,1);
    draw_right_arm(b_shoulders_hip_r,b_height-2*b_head_r-b_neck+b_neck/3-b_shoulders_hip_r,0,1);
    draw_chest(b_height-2*b_head_r-b_neck+b_neck/3-b_shoulders_hip_r-b_chest/2,1);
    draw_hip(b_height-2*b_head_r-b_neck+b_neck/3-b_shoulders_hip_r-2*b_chest/2);
  }
}


//stroke(255, 0, 0);
//line(-1000, 0, 0, 1000, 0, 0);
//stroke(0, 255, 0);
//line(0, -1000, 0, 0, 1000, 0);
//stroke(0, 0, 255);
//line(0, 0, -1000, 0, 0, 1000);
