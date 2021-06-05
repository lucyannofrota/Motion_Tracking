//PGraphics GraphsFB;
GraphsC Graphs;

final int DataVectorSize = 10000;

class GraphsC{
  
  int DrawSize[];
  
  public PGraphics IBuf;
  
  GraphsC(int [] Dsize){
    //super();
    DrawSize = Dsize;
    //println(DrawSize);
    IBuf = createGraphics(DrawSize[0],DrawSize[1],P2D);
  }
  
  PGraphics draw(){
    //IBuf.beginDraw();
    ////#676767
    //IBuf.background(#FF0808);
    //IBuf.endDraw();
    
    IBuf.beginDraw();
    IBuf.background(51);
    IBuf.noFill();
    IBuf.stroke(255);
    IBuf.ellipse(mouseX-120, mouseY-60, 60, 60);
    IBuf.endDraw();
    return IBuf;
  }
}
