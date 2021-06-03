class IMU{
  public float ax,ay,az;
  public float gx,gy,gz;
  
  IMU(){
    ax = 0; ay = 0; az = 0;
    gx = 0; gy = 0; gz = 0;
  }
  
  IMU(float ax_, float ay_, float az_, float gx_, float gy_, float gz_){
    ax = ax_; ay = ay_; az = az_;
    gx = gx_; gy = gy_; gz = gz_;
  }
  
  void def(float ax_, float ay_, float az_, float gx_, float gy_, float gz_){
    ax = ax_; ay = ay_; az = az_;
    gx = gx_; gy = gy_; gz = gz_;
  }
}
