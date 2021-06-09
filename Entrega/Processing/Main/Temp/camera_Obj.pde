class camera_Obj{
  float x; float y; float z;
  float pitch; float yaw;
  float fov;
  
  camera_Obj(float x_,float y_, float z_){
    this.x = x_;
    this.y = y_;
    this.z = z_;
    this.pitch = 0;
    this.yaw = radians(-90);
    this.fov = 1.5; 
  }
  
  void transl(float dXm, float dYm){
    cam.x+=dXm/3;
    cam.y+=dYm/3;
    this.update();
  }
  
  void rot(float dXm, float dYm){
    cam.yaw-=dXm/500;
    if(cam.yaw > 89){
      cam.yaw = 89;
    }
    if(cam.yaw < -89){
      cam.yaw = -89;
    }
    cam.pitch-=dYm/500;
    if(cam.pitch > 89){
      cam.pitch = 89;
    }
    if(cam.pitch < -89){
      cam.pitch = -89;
    }
    
    this.update();
  }
  
  void update(){
    camera(x, y, z, x+cos(yaw)*cos(pitch), y+sin(pitch), z+sin(yaw)*cos(pitch), 0, -1, 0);
    perspective(fov, float(width)/float(height), 0.001, 1000000);
  }
}
//let Ry = Math.sin(glm.radians(this.pitch));
//        let Rz = Math.sin(glm.radians(this.yaw))*Math.cos(glm.radians(this.pitch));


void mouseDragged() 
{
  float dXm = mouseX-pmouseX, dYm = mouseY-pmouseY;
  switch(mouseButton){
    case LEFT:
      cam.transl(dXm,dYm);
    break;
    case RIGHT:
      cam.rot(dXm,dYm);
    break;
  }
}

void mouseWheel(MouseEvent event) {
  //cam.z+=event.getCount()*20;
  //cam.x+cos(cam.yaw)*cos(pitch), y+sin(pitch), z+sin(yaw)*cos(pitch)
  cam.fov+=float(event.getCount())/10;
  //println(cam.fov);
  if(cam.fov < 1){
    cam.fov = 1;
  }
  if(cam.fov > 2){
    cam.fov = 2;
  }
  cam.update();
}
