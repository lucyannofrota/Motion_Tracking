
PGraphics Sensor3DFB;
Sensor3DC Sensor3D;

class Sensor3DC extends PGraphics{
  
  float DrawSize[];
  
  Sensor3DC(float [] Dsize){
    super();
    DrawSize = Dsize;
  }
  
  void draw(){
    this.beginDraw();
    this.background(#676767);
    this.endDraw();
  }
}
