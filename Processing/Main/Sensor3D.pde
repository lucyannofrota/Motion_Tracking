
//PGraphics Sensor3DFB;
Sensor3DC Sensor3D;

class Sensor3DC extends PGraphics{
  
  int DrawSize[];
  
  PGraphics IBuf;
  
  Sensor3DC(int [] Dsize){
    DrawSize = Dsize;
    IBuf = createGraphics(DrawSize[0],DrawSize[1],P3D);
  }
  
  PGraphics draw(){
    //cam.update();
    IBuf.beginDraw();
    IBuf.background(#676767);
    IBuf.fill(#F70A0A);
    IBuf.box(70);
    IBuf.lights();
    IBuf.endDraw();
    return IBuf;
  }
}
