class Pose{
  float X = 0, Y = 0, Z = 0;
  float Rx = 0, Ry = 0, Rz = 0;
  
  Pose(float x_, float y_, float z_, float rx_, float ry_, float rz_){
    this.X = x_; this.Y = y_; this.Z = z_;
    this.Rx = rx_; this.Ry = ry_; this.Rz = rz_;
  }
  
  void setZero(){
    this.X = 0; this.Y = 0; this.Z = 0;
    this.Rx = 0; this.Ry = 0; this.Rz = 0;
  }
  
  void print(){
    println("Pose: \nT -> ("+this.X+','+this.Y+','+this.Z+")\nR -> ("+this.Rx+','+this.Ry+','+this.Rz+")\n");
  }
}
