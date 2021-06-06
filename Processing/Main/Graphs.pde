//PGraphics GraphsFB;
GraphsC Graphs;

final int DataVectorSize = 10000;

class GraphsC {

  int DrawPosition[];
  int DrawSize[];
  PGraphics IBuf;

  int time = 0; //time in ms

  public PlotGroupIMU Sensor1;
  PlotGroupIMU Sensor2;
  PlotGroupIMU Sensor3;
  PlotGroupIMU Sensor4;

  boolean flag;
  
  final protected PApplet parent;

  GraphsC(PApplet _parent,int [] Dpos,int [] Dsize) {
    parent = _parent;
    DrawPosition = Dpos;
    DrawSize = Dsize;
    IBuf = createGraphics(DrawSize[0], DrawSize[1], P2D);

    flag = false;


    time = 0;

    // Criando os plots


    Sensor1 = new PlotGroupIMU(parent,float(Dpos),new float[]{DrawSize[0]-4,DrawSize[1]-4});
    //Sensor2 = new PlotGroupIMU(parent,float(Dpos),new float[]{DrawSize[0]-4,DrawSize[1]-4},IBuf);
    //Sensor3 = new PlotGroupIMU(parent,float(Dpos),new float[]{DrawSize[0]-4,DrawSize[1]-4},IBuf);
    //Sensor4 = new PlotGroupIMU(parent,float(Dpos),new float[]{DrawSize[0]-4,DrawSize[1]-4},IBuf);
    
    IBuf.beginDraw();
    IBuf.background(#676767);
    IBuf.endDraw();
  }

  void draw(float w,float h) {
    image(IBuf,w,h);
    Sensor1.draw();
  }


  void mouseClicked() {
    flag = !flag;
    Sensor1.toggleInd();
  }

  public void newData(IMU sens, int nS) {
    int elt;
    if (time == 0) {
      time = millis();
      elt = 0;
    } else {
      elt = millis() - time;
    }
    switch(nS) {
    case 1:
      Sensor1.addPoint(elt/1000.0, sens);
      break;
    case 2:
      Sensor2.addPoint(elt/1000.0, sens);
      break;
    case 3:
      Sensor3.addPoint(elt/1000.0, sens);
      break;
    case 4:
      Sensor4.addPoint(elt/1000.0, sens);
      break;
    }
  }








  // Conjunto graficos
  class PlotGroupIMU {
    myGPlot ax, ay, az;
    myGPlot rx, ry, rz;

    int Npoints = 0;
    final int NdataPlot = 100;
    
    boolean Cflag = false ,Dflag = false;

    PlotGroupIMU(PApplet parent,float [] _posis,float [] _dimens) {
      //final float Width = _dimens[0], Height = _dimens[1];
      //final float Width = 1920, Height = 1080;
      
      final float IPos[] = {_posis[0]+2,_posis[1]+3};

      final float wid = (_dimens[0]-2)/2;
      final float hei = (_dimens[1])/3;

      ax = new myGPlot(parent);
      ax.setTitleText("Acceleration X");
      ax.setPositionDim(IPos[0],IPos[1]+0*hei, wid, hei);
      // println("PlotIMG");
      // println(wid,hei);
      ax.toggleInd();
      ax.setYLim(-0.5,0.5);
      ax.setINDPosf(new String[]{"g"});

      ay = new myGPlot(parent);
      ay.setTitleText("Acceleration Y");
      ay.setPositionDim(IPos[0]+0*wid, IPos[1]+1*hei, wid, hei);
      ay.toggleInd();
      ay.setYLim(-0.5,0.5);
      ax.setINDPosf(new String[]{"g"});

      az = new myGPlot(parent, true);
      az.setTitleText("Acceleration Z");
      az.setPositionDim(IPos[0]+0*wid, IPos[1]+2*hei, wid, hei);
      az.toggleInd();
      az.setYLim(-0.5,0.5);
      ax.setINDPosf(new String[]{"g"});

      rx = new myGPlot(parent);
      rx.setTitleText("Gyro X");
      rx.setPositionDim(IPos[0]+1*wid+2, IPos[1]+0*hei, wid, hei);
      rx.toggleInd();
      rx.setYLim(-180,180);
      ax.setINDPosf(new String[]{"º"});

      ry = new myGPlot(parent);
      ry.setTitleText("Gyro Y");
      ry.setPositionDim(IPos[0]+1*wid+2, IPos[1]+1*hei, wid, hei);
      ry.toggleInd();
      ry.setYLim(-90,90);
      ax.setINDPosf(new String[]{"º"});

      rz = new myGPlot(parent, true);
      rz.setTitleText("Gyro Z");
      rz.setPositionDim(IPos[0]+1*wid+2, IPos[1]+2*hei, wid, hei);
      rz.toggleInd();
      rz.setYLim(-180,180);
      ax.setINDPosf(new String[]{"º"});
    }

    void addPoint(float time, IMU sens) {
      while(Dflag) delay(1);
      Cflag = true;
      Npoints++;
      ax.addPoint(new GPoint(time, sens.ax));
      ay.addPoint(new GPoint(time, sens.ay));
      az.addPoint(new GPoint(time, sens.az));
      rx.addPoint(new GPoint(time, sens.rx));
      ry.addPoint(new GPoint(time, sens.ry));
      rz.addPoint(new GPoint(time, sens.rz));
      if (Npoints >= NdataPlot) {
        Npoints--;
        ax.removePoint(0);
        ay.removePoint(0);
        az.removePoint(0);
        rx.removePoint(0);
        ry.removePoint(0);
        rz.removePoint(0);
      }
      Cflag = false;
    }


    void draw() {
      while(Cflag) delay(1);
      Dflag = true;
      ax.Draw(false);
      ay.Draw(false);
      az.Draw(true);
      rx.Draw(false);
      ry.Draw(false);
      rz.Draw(true);
      Dflag = false;
    }


    void toggleInd() {
      ax.toggleInd();
      ay.toggleInd();
      az.toggleInd();
      rx.toggleInd();
      ry.toggleInd();
      rz.toggleInd();
    }








    // Graficos per se
    class myGPlot extends GPlot {
      final float Height = 1080, Width = 1920;
      float GraphHeight = floor(Height*0.25);
      float GraphWidth = floor((Width*0.98/2));
      final float IndHeight;
      final float IndWidtht = 40;
      //final int GraphWidthIND;
      boolean hasInd;
      //final int plotIntSpace = 0;
      int martop = 18, marbot=5, marright = 5, marleft = 30;

      final int bord = 10;
      
      PGraphics IBuf;

      String INDposf[] = {""};

      IND indicator;
      myGPlot(PApplet parent) {
        super(parent);
        getTitle().setOffset(1);
        setPos(5, 100);
        setMar(marbot, marleft, martop, marright);
        setOuterDim(GraphWidth, GraphHeight);
        IndHeight = dim[1];
        //GraphWidthIND = floor((Width*0.98/2)-IndWidtht);
        hasInd = false;
        indicator = new IND(parent, new float[]{IndWidtht, IndHeight});
        //float dim[] = getDim();
        // indicator.setPDims(dim[0], 0, IndWidtht, IndHeight);
      }

      myGPlot(PApplet parent,boolean f) {
        super(parent);
        if (f) marbot = 30;
        getTitle().setOffset(1);
        setPos(5, 100);
        setMar(marbot, marleft, martop, marright);
        setOuterDim(GraphWidth, GraphHeight);
        IndHeight = dim[1];
        //GraphWidthIND = floor((Width*0.98/2)-IndWidtht);
        hasInd = false;
        indicator = new IND(parent, new float[]{IndWidtht, IndHeight});
        // indicator.setPDims(dim[0], 0, IndWidtht, IndHeight);
      }

      void setPositionDim(float x, float y, float dimX, float dimY) {
        GraphWidth = dimX; 
        GraphHeight = dimY;
        setPos(x, y);
        setOuterDim(dimX, dimY);
        // println(dimX, dimY);
        if(hasInd){
          indicator.setPDims(x+GraphWidth, y+GraphHeight,dimX,dimY);
        }
        //println(x+GraphHeight, y+GraphWidth);
      }

      void setINDPosf(String [] str){
        INDposf = str;
      }

      void toggleInd() {
        hasInd = !hasInd;
        if (hasInd) {
          //IndWidtht
          setDim(dim[0]-IndWidtht, dim[1]);
          setMar(marbot, marleft, martop, marright+IndWidtht);
          //setOuterDim(GraphWidthIND, GraphHeight);
          indicator.setPDims(dim[0], 0, IndWidtht, dim[1]);
        } else {
          setDim(dim[0], dim[1]);
          setMar(marbot, marleft, martop, marright);
          setOuterDim(GraphWidth, GraphHeight);
        }
      }

      public void drawBackground() {
        parent.pushStyle();
        parent.rectMode(CORNER);
        parent.fill(bgColor);
        parent.noStroke();
        if (hasInd) parent.rect(-mar[1], -mar[2] - dim[1], outerDim[0], outerDim[1], 0, bord, bord, 0);
        else parent.rect(-mar[1], -mar[2] - dim[1], outerDim[0], outerDim[1]);
        parent.popStyle();
      }

      public void drawBox() {
        parent.pushStyle();
        parent.rectMode(CORNER);
        parent.fill(boxBgColor);
        parent.stroke(boxLineColor);
        parent.strokeWeight(boxLineWidth);
        parent.strokeCap(SQUARE);
        if (hasInd) parent.rect(0, -dim[1], dim[0], dim[1], 0, bord, bord, 0);
        else parent.rect(0, -dim[1], dim[0], dim[1], 0, 0, 0, 0);
        parent.popStyle();
      }

      public void addPoint(GPoint newPoint) {
        mainLayer.addPoint(newPoint);
        //updateLimits();
        //new String [] = {concat(str(newPoint.getY()),INDposf)};
        indicator.update(str(newPoint.getY()));
      }

      void Draw(boolean X) {
        beginDraw();
        drawBackground();
        drawBox();
        if(X) drawXAxis();
        drawYAxis();
        //plot.drawTopAxis();
        //plot.drawRightAxis();
        drawTitle();
        drawLines();
        //try {
        //  drawLines();
        //}
        //catch(IndexOutOfBoundsException e) {
        //  e.printStackTrace();
        //}
        //catch(NullPointerException e) {
        //  e.printStackTrace();
        //}
        if (hasInd) indicator.draw();
        endDraw();
      }







      // Caixinha verde
      class IND {
        float Height = 80;
        float Widtht = 80;
        float posX = 30;
        float posY = 30;
        final int bordRad = 5;
        protected final PApplet parent;

        String text = "0.0";

        final color fillc = color(255, 255, 255), lienc = color(0, 255, 0);
        float lineWidth = 1;
        IND(PApplet parent_, float [] siz) {
          Widtht = siz[1];
          Height = siz[1];
          parent = parent_;
        }

        void setPDims(float x, float y, float dimX, float dimY) {
          posX = x; 
          posY = y;
          Widtht = dimX;
          Height = dimY;
        }

        void update(String txt) {
          text = txt;
        }

        void draw() {
          parent.pushStyle();
          parent.rectMode(CORNER);
          parent.fill(fillc);
          parent.stroke(lienc);
          parent.strokeWeight(lineWidth);
          parent.strokeCap(SQUARE);
          pushMatrix();
          translate(posX+2, posY);
          parent.rect(0, -Height, Widtht, Height, bordRad, bordRad, bordRad, bordRad);
          drawText();
          popMatrix();
          parent.popStyle();
        }

        void drawText() {
          parent.pushStyle();
          textAlign(CENTER, CENTER);
          fill(0, 102, 200, 100);
          //stroke(153);
          //textSize(11);
          text(text, 0, -Height, Widtht, Height);
          parent.popStyle();
        }
      }
    }
  }
}
