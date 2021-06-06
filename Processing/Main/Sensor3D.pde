
//PGraphics Sensor3DFB;
Sensor3DC Sensor3D;

class Sensor3DC extends PGraphics {

  int DrawSize[];
  Plane pl;
  PGraphics IBuf;

  Sensor3DC(int [] Dsize) {
    DrawSize = Dsize;
    IBuf = createGraphics(DrawSize[0], DrawSize[1], P3D);
    pl = new Plane(IBuf, sensor);
  }

  PGraphics draw() {
    IBuf.beginDraw();
    IBuf.noStroke();
    drawBackground();
    //IBuf.translate(width/2, height/2);
    IBuf.lights();
    pl.display();
    IBuf.endDraw();
    return IBuf;
  }
  void drawBackground() {
    PImage img = loadImage("sky.jpg");
    IBuf.beginShape();
    IBuf.textureMode(NORMAL);
    IBuf.texture(img);
    IBuf.vertex(-1000, -500, -800, 0, 0);
    IBuf.vertex(3000, -500, -800, 1, 0);
    IBuf.vertex(3000, 2000, -800, 1, 1);
    IBuf.vertex(-1000, 2000, -800, 0, 1);
    IBuf.endShape(CLOSE);
  }





  void mouseClicked() {
    //flag = !flag;
    //Sensor1.toggleInd();
  }
  
  
  void mouseWheel(MouseEvent e){
    
  } 

  void mouseDragged(){
    
  }




}



class Plane {
    PGraphics d; 
    IMU sensor;
    Plane(PGraphics ibuf, IMU sens) {
      d = ibuf;
      sensor = sens;
    }

    void display() {
      d.pushMatrix();
      d.translate(width/2, height/2, -250);
      d.rotateX(radians(sensor.rx));
      d.rotateZ(radians(sensor.ry));
      d.rotateY(radians(sensor.rz));
      d.scale(200);
      d.scale(1, 1, 1.2);
      // body
      d.pushMatrix();
      d.fill(#5AE5CB);
      d.scale(0.6, 0.5, 1.3);
      d.sphere(1); 

      d.popMatrix();

      // ball
      d.pushMatrix();
      d.scale(0.5);
      d.fill(0, 100, 200);
      d.translate(0.0, -0.35, 0.45);
      d.sphere(1);
      d.popMatrix();

      //wings
      d.pushMatrix();
      d.fill(100, 0, 100);
      d.translate(0, 0.1, 0.25);
      d.scale(1.6, 0.2, 0.5);
      d.sphere(1); 
      d.popMatrix();

      //coiso para cima
      d.pushMatrix();
      d.fill(100, 0, 100);
      d.translate(0, -0.25, -1);
      d.scale(0.1, 0.4, 0.3);
      d.sphere(1); 
      d.popMatrix();

      //coiso para os lados
      d.pushMatrix();
      d.fill(100, 0, 100);
      d.translate(0, 0, -0.95);
      d.scale(0.8, 0.22, 0.35);
      d.sphere(1); 
      d.popMatrix();
      d.popMatrix();
    }
  }
