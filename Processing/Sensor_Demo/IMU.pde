class IMU{
  public float ax,ay,az;
  public float rx,ry,rz;
  
  IMU(){
    ax = 0; ay = 0; az = 0;
    rx = 0; ry = 0; rz = 0;
  }
  
  IMU(float ax_, float ay_, float az_, float rx_, float ry_, float rz_){
    ax = ax_; ay = ay_; az = az_;
    rx = rx_; ry = ry_; rz = rz_;
  }
  
  void def(float ax_, float ay_, float az_, float rx_, float ry_, float rz_){
    ax = ax_; ay = ay_; az = az_;
    rx = rx_; ry = ry_; rz = rz_;
  }
}
