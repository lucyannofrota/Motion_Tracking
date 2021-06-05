camera_Obj cam;

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
    if(currentTAB == _TAB.BODY){
      camera(x, y, z, x+cos(yaw)*cos(pitch), y+sin(pitch), z+sin(yaw)*cos(pitch), 0, -1, 0);
      perspective(fov, float(width)/float(height), 0.001, 1000000);
    }
  }
}
//let Ry = Math.sin(glm.radians(this.pitch));
//        let Rz = Math.sin(glm.radians(this.yaw))*Math.cos(glm.radians(this.pitch));
