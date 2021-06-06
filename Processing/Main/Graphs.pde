//PGraphics GraphsFB;
GraphsC Graphs;

final int DataVectorSize = 10000;



class GraphsC {

  int DrawPosition[];
  int DrawSize[];
  PGraphics IBuf;

  int time = 0; //time in ms

  public PlotGroupIMU Sensor1;
  //PlotGroupIMU Sensor2;
  //PlotGroupIMU Sensor3;
  //PlotGroupIMU Sensor4;

  //boolean flag;

  final protected PApplet parent;



  GraphsC(PApplet _parent, int [] Dpos, int [] Dsize) {
    parent = _parent;
    DrawPosition = Dpos;
    DrawSize = Dsize;
    IBuf = createGraphics(DrawSize[0], DrawSize[1], P2D);

    //flag = false;

    time = 0;

    // Criando os plots
    color bgColor = color(#25251d);



    Sensor1 = new PlotGroupIMU(parent, float(Dpos), new float[]{DrawSize[0]-4, DrawSize[1]-4});
    //Sensor2 = new PlotGroupIMU(parent,float(Dpos),new float[]{DrawSize[0]-4,DrawSize[1]-4},IBuf);
    //Sensor3 = new PlotGroupIMU(parent,float(Dpos),new float[]{DrawSize[0]-4,DrawSize[1]-4},IBuf);
    //Sensor4 = new PlotGroupIMU(parent,float(Dpos),new float[]{DrawSize[0]-4,DrawSize[1]-4},IBuf);

    IBuf.beginDraw();
    IBuf.background(bgColor);
    IBuf.endDraw();
  }

  void draw(float w, float h) {
    image(IBuf, w, h);
    Sensor1.draw();
  }


  void mouseClicked() {
    //flag = !flag;
    Sensor1.toggleInd();
  }


  void mouseWheel(MouseEvent e) {
  } 

  void mouseDragged() {
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
      //Sensor2.addPoint(elt/1000.0, sens);
      break;
    case 3:
      //Sensor3.addPoint(elt/1000.0, sens);
      break;
    case 4:
      //Sensor4.addPoint(elt/1000.0, sens);
      break;
    }
  }

  float[] getRPY(int Nsens) {
    float [] rpy;
    rpy = Sensor1.getRPY();
    //switch(Nsens){
    //  case 1:
    //    rpy = Sensor1.getRPY();
    //  break;
    //}
    return rpy;
  }


  PlotGroupIMU getSens(int Nsens) {
    return Sensor1;
  }
}


// Conjunto graficos
class PlotGroupIMU {
  myGPlot ax, ay, az;
  myGPlot rx, ry, rz;

  int Npoints = 0;
  final int NdataPlot = 100;
  color lineColor = color(#9763CB);  
  boolean Cflag = false, Dflag = false;

  color bgColor = color(#25251d);
  color boxBgColor = color(#3a3b3c);
  color boxLineColor = color(#204a42);
  
  IMU tempBuf;

  PlotGroupIMU(PApplet parent, float [] _posis, float [] _dimens) {
    //final float Width = _dimens[0], Height = _dimens[1];
    //final float Width = 1920, Height = 1080;
    tempBuf = new IMU();
    final float IPos[] = {_posis[0]+2, _posis[1]+3};

    final float wid = (_dimens[0]-2)/2;
    final float hei = (_dimens[1])/3;

    ax = new myGPlot(parent);
    ax.setTitleText("Acceleration X");
    ax.setPositionDim(IPos[0], IPos[1]+0*hei, wid, hei);
    ax.toggleInd();
    ax.setLineColor(lineColor);
    ax.setYLim(-1, 1);
    ax.setINDPosf(new String[]{"g"});

    ay = new myGPlot(parent);
    ay.setTitleText("Acceleration Y");
    ay.setPositionDim(IPos[0]+0*wid, IPos[1]+1*hei, wid, hei);
    ay.toggleInd();
    ay.setYLim(-1, 1);
    ay.setLineColor(lineColor);
    ay.setINDPosf(new String[]{"g"});

    az = new myGPlot(parent, true);
    az.setTitleText("Acceleration Z");
    az.setPositionDim(IPos[0]+0*wid, IPos[1]+2*hei, wid, hei);
    az.toggleInd();
    az.setYLim(-1, 1);
    az.setLineColor(lineColor);
    az.setINDPosf(new String[]{"g"});

    rx = new myGPlot(parent);
    rx.setTitleText("Gyro X");
    rx.setPositionDim(IPos[0]+1*wid+2, IPos[1]+0*hei, wid, hei);
    rx.toggleInd();
    rx.setYLim(-200, 200);
    rx.setLineColor(lineColor);
    rx.setINDPosf(new String[]{"º"});

    ry = new myGPlot(parent);
    ry.setTitleText("Gyro Y");
    ry.setPositionDim(IPos[0]+1*wid+2, IPos[1]+1*hei, wid, hei);
    ry.toggleInd();
    ry.setYLim(-100, 100);
    ry.setLineColor(lineColor);
    ry.setINDPosf(new String[]{"º"});

    rz = new myGPlot(parent, true);
    rz.setTitleText("Gyro Z");
    rz.setPositionDim(IPos[0]+1*wid+2, IPos[1]+2*hei, wid, hei);
    rz.toggleInd();
    rz.setLineColor(lineColor);
    rz.setYLim(-200, 200);
    rz.setINDPosf(new String[]{"º"});
  }

  void addPoint(float time, IMU sens) {
    while (Dflag) delay(1);
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
    tempBuf.def(sens.ax,sens.ay,sens.az,sens.rx,sens.ry,sens.rz);
    Cflag = false;
  }

  float[] getRPY() {
    //if(InitializationFlag == false) return new float[]{0, 0, 0};
    //while (Cflag) delay(1);
    float [] ret = new float[]{tempBuf.rx, tempBuf.ry, tempBuf.rz};
    return ret;
  }


  void draw() {
    while (Cflag) delay(1);
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
    final float IndWidtht = 70;
    //final int GraphWidthIND;
    boolean hasInd;
    //final int plotIntSpace = 0;
    int martop = 18, marbot=5, marright = 5, marleft = 30;

    final int bord = 10;

    PGraphics IBuf;

    String INDposf[] = {""};

    IND indicator;

    color bgColor = color(#25251d);
    color boxBgColor = color(#3a3b3c);
    color boxLineColor = color(#204a42);
    color axisColor = color(#453d3c);
    myGPlot(PApplet parent) {
      super(parent);
      getTitle().setOffset(1);
      getTitle().setFontColor(lineColor);
      getXAxis().setFontColor(lineColor);
      getYAxis().setFontColor(lineColor);
      getXAxis().setLineColor(axisColor);
      getYAxis().setLineColor(axisColor);
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

    myGPlot(PApplet parent, boolean f) {
      super(parent);
      if (f) marbot = 30;
      getTitle().setOffset(1);
      getTitle().setFontColor(lineColor);
      getXAxis().setFontColor(lineColor);
      getYAxis().setFontColor(lineColor);
      getXAxis().setLineColor(axisColor);
      getYAxis().setLineColor(axisColor);
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
      if (hasInd) {
        indicator.setPDims(x+GraphWidth, y+GraphHeight, dimX, dimY);
      }
      //println(x+GraphHeight, y+GraphWidth);
    }

    void setINDPosf(String [] str) {
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
      if (hasInd) parent.rect(-mar[1], -mar[2] - dim[1], outerDim[0], outerDim[1], bord);
      else parent.rect(-mar[1], -mar[2] - dim[1], outerDim[0], outerDim[1], bord);
      parent.popStyle();
    }

    public void drawBox() {
      parent.pushStyle();
      parent.rectMode(CORNER);
      parent.fill(boxBgColor);
      parent.noStroke();
      //parent.stroke(boxLineColor);
      //parent.strokeWeight(boxLineWidth);
      parent.strokeCap(SQUARE);
      if (hasInd) parent.rect(0, -dim[1], dim[0], dim[1], bord);
      else parent.rect(0, -dim[1], dim[0], dim[1], bord);
      parent.popStyle();
    }

    public void addPoint(GPoint newPoint) {
      mainLayer.addPoint(newPoint);
      //updateLimits();
      String temp = str(newPoint.getY());
      String temp2 = INDposf[0];
      String arr = temp.concat(temp2);
      indicator.update(arr);
    }

    void Draw(boolean X) {
      beginDraw();
      drawBackground();
      drawBox();
      if (X) drawXAxis();
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
      //color lineColor = color(#16b694);

      color boxBgColor = color(#453d3c);
      String text = "0.0";
      //final color bgcolor = color(#575aa7);
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
        parent.strokeCap(SQUARE);
        pushMatrix();
        noStroke();
        translate(posX+2, posY);
        fill(boxBgColor);
        parent.rect(0, -Height, Widtht, Height, bordRad);
        drawText();
        popMatrix();
        parent.popStyle();
      }

      void drawText() {
        parent.pushStyle();
        textAlign(CENTER, CENTER);

        parent.fill(lineColor);

        text(text, 0, -Height, Widtht, Height);
        parent.popStyle();
      }
    }
  }
}
