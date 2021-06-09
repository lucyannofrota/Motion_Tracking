public class myGraphWindow extends PApplet {

  int time = 0; //time in ms

  GPlot mainPlot;
  GPlot mainPlot2;


  //PlotGroupIMU compPlot;
  PlotGroupIMU Sensor1;
  PlotGroupIMU Sensor2;
  PlotGroupIMU Sensor3;
  PlotGroupIMU Sensor4;

  boolean flag;

  myGraphWindow() {
    super();
    PApplet.runSketch(new String[]{this.getClass().getName()}, this);
  }

  public void settings() {
    //size(1920, 1080);
    fullScreen();
    flag = false;
  }

  public void setup() {
    time = 0;
    // Nome da janela
    surface.setTitle("Graphs");

    // Criando o plot central
    mainPlot = new GPlot(this);
    mainPlot.setPos(100, 0);
    mainPlot.setDim(1600, 300);
    mainPlot2 = new GPlot(this);
    mainPlot2.setPos(100, 0);
    mainPlot2.setDim(1600, 300);

    // Criando o vetor de pontos
    GPointsArray points1 = new GPointsArray(DataVectorSize);
    GPointsArray points2 = new GPointsArray(DataVectorSize);
    for (int i = 0; i < 1000; i++) {
      points1.add(new GPoint(i, pow(i, 1.5)));
      points2.add(new GPoint(i, pow(1000-i, 1.5)));
    }
    mainPlot.setPoints(points1);
    mainPlot2.setPoints(points2);
    GPointsArray temp1 = mainPlot.getMainLayer().getPointsRef();
    //println(temp1.getLastPoint().getY());
    GPointsArray temp2 = mainPlot2.getMainLayer().getPointsRef();
    for (int i = 1001; i < 2000; i++) {
      temp1.add(new GPoint(i, pow(i, 1.5)));
      temp2.add(new GPoint(i, pow(2000-i, 1.5)));
    }
    //println(mainPlot.getMainLayer().getPointsRef().getLastPoint().getY());
    //for(int i = 1001; i < 2000; i++){
    //  points1.add(new GPoint(i,pow(i,1.5)));
    //  points2.add(new GPoint(i,pow(2000-i,1.5)));
    //}
    mainPlot.getMainLayer().setPointColor(color(100, 100, 100));




    // Codigo a cima é apenas uma demo


    // Criando os plots


    Sensor1 = new PlotGroupIMU(this);
    Sensor2 = new PlotGroupIMU(this);
    Sensor3 = new PlotGroupIMU(this);
    Sensor4 = new PlotGroupIMU(this);


    //for (int i = 0; i < 100; i++) {
    //  Sensor1.addPoint(i, new IMU((80/(i+1))*sin(i*PI/4), 90*cos(i*PI/8), 0, 0, 0, 0));
    //}

    //mainPlot = new GPlot(this);
    //mainPlot.setPos(100, 0);
    //mainPlot.setDim(1600, 300);
    //mainPlot2 = new GPlot(this);
    //mainPlot2.setPos(100, 0);
    //mainPlot2.setDim(1600, 300);
  }

  public void draw() {
    //if(time == 0) return; Habilitar quando houver uma timeseries
    background(150);
    Sensor1.draw();
    /*
    if(flag == false){
     mainPlot.beginDraw();
     mainPlot.drawBackground();
     mainPlot.drawBox();
     mainPlot.drawXAxis();
     mainPlot.drawYAxis();
     mainPlot.drawTopAxis();
     mainPlot.drawRightAxis();
     mainPlot.drawTitle();
     mainPlot.getMainLayer().drawPoints();
     //mainPlot.getLayer("Lay2").drawPoints();
     mainPlot.endDraw();
     }
     else{
     mainPlot2.beginDraw();
     mainPlot2.drawBackground();
     mainPlot2.drawBox();
     mainPlot2.drawXAxis();
     mainPlot2.drawYAxis();
     mainPlot2.drawTopAxis();
     mainPlot2.drawRightAxis();
     mainPlot2.drawTitle();
     mainPlot2.getMainLayer().drawPoints();
     //mainPlot.getLayer("Lay2").drawPoints();
     mainPlot2.endDraw();
     }*/
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
      Sensor1.addPoint(elt, sens);
      break;
    case 2:
      Sensor2.addPoint(elt, sens);
      break;
    case 3:
      Sensor3.addPoint(elt, sens);
      break;
    case 4:
      Sensor4.addPoint(elt, sens);
      break;
    }
  }


  class PlotGroupIMU {
    myGPlot ax, ay, az;
    myGPlot rx, ry, rz;
    
    int Npoints = 0;
    final int NdataPlot = 100;

    PlotGroupIMU(PApplet parent) {
      final float Width = 1920, Height = 1080;
      
      final float wid = (Width-10-4)/2;
      final float hei = (Height-10)/4;
      
      ax = new myGPlot(parent);
      ax.setTitleText("Acceleration X");
      ax.setPositionDim(5+0*wid,1*hei+5,wid,hei);
      ax.toggleInd();
      
      ay = new myGPlot(parent);
      ay.setTitleText("Acceleration Y");
      ay.setPositionDim(5+0*wid,2*hei+5,wid,hei);
      ay.toggleInd();
      
      az = new myGPlot(parent,true);
      az.setTitleText("Acceleration Z");
      az.setPositionDim(5+0*wid,3*hei+5-1,wid,hei);
      az.toggleInd();
      
      rx = new myGPlot(parent);
      rx.setTitleText("Gyro X");
      rx.setPositionDim(5+1*wid+4,1*hei+5,wid,hei);
      rx.toggleInd();
      
      ry = new myGPlot(parent);
      ry.setTitleText("Gyro Y");
      ry.setPositionDim(5+1*wid+4,2*hei+5,wid,hei);
      ry.toggleInd();
      
      rz = new myGPlot(parent,true);
      rz.setTitleText("Gyro Z");
      rz.setPositionDim(5+1*wid+4,3*hei+5-1,wid,hei);
      rz.toggleInd();
    }

    void addPoint(int time, IMU sens) {
      Npoints++;
      ax.addPoint(new GPoint(time, sens.ax));
      ay.addPoint(new GPoint(time,sens.ay));
      az.addPoint(new GPoint(time,sens.az));
      rx.addPoint(new GPoint(time,sens.rx));
      ry.addPoint(new GPoint(time,sens.ry));
      rz.addPoint(new GPoint(time,sens.rz));
      if(Npoints >= NdataPlot){
        Npoints--;
        ax.removePoint(0);
        ay.removePoint(0);
        az.removePoint(0);
        rx.removePoint(0);
        ry.removePoint(0);
        rz.removePoint(0);
      }
    }


    void draw() {
      ax.Draw();
      ay.Draw();
      az.Draw();
      rx.Draw();
      ry.Draw();
      rz.Draw();
      //drawPlot(ay);
      //drawPlot(az);
      //drawPlot(rx);
      //drawPlot(ry);
      //drawPlot(rz);
    }

    void drawPlot(GPlot plot) {
      plot.beginDraw();
      plot.drawBackground();
      plot.drawBox();
      //plot.drawXAxis();
      plot.drawYAxis();
      //plot.drawTopAxis();
      //plot.drawRightAxis();
      plot.drawTitle();
      plot.getMainLayer().drawLines();
      plot.endDraw();
    }
    
    void toggleInd(){
      ax.toggleInd();
      ay.toggleInd();
      az.toggleInd();
      rx.toggleInd();
      ry.toggleInd();
      rz.toggleInd();
    }
  }

  class myGPlot extends GPlot {
    final float Height = 1080, Width = 1920;
    float GraphHeight = floor(Height*0.25);
    float GraphWidth = floor((Width*0.98/2));
    final float IndHeight;
    final float IndWidtht = 40;
    //final int GraphWidthIND;
    boolean hasInd;
    //final int plotIntSpace = 0;
    int martop = 15, marbot=5, marright = 5, marleft = 30;

    final int bord = 10;

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
      indicator = new IND(parent,new float[]{IndWidtht,IndHeight});
      //float dim[] = getDim();
      indicator.setPDims(dim[0], 0,IndWidtht,IndHeight);
      //println(5+dim[0], 0);
    }
    
    myGPlot(PApplet parent,boolean f) {
      super(parent);
      if(f) marbot = 30;
      getTitle().setOffset(1);
      setPos(5, 100);
      setMar(marbot, marleft, martop, marright);
      setOuterDim(GraphWidth, GraphHeight);
      IndHeight = dim[1];
      //GraphWidthIND = floor((Width*0.98/2)-IndWidtht);
      hasInd = false;
      indicator = new IND(parent,new float[]{IndWidtht,IndHeight});
      //float dim[] = getDim();
      indicator.setPDims(dim[0], 0,IndWidtht,IndHeight);
      //println(5+dim[0], 0);
    }

    void setPositionDim(float x, float y,float dimX, float dimY) {
      GraphWidth = dimX; 
      GraphHeight = dimY;
      setPos(x, y);
      setOuterDim(dimX,dimY);
      //indicator.setPDims(x+GraphWidth, y+GraphHeight,IndWidtht,IndHeight);
      //println(x+GraphHeight, y+GraphWidth);
    }
    
    void toggleInd() {
      hasInd = !hasInd;
      if (hasInd) {
        //IndWidtht
        setDim(dim[0]-IndWidtht,dim[1]);
        setMar(marbot, marleft, martop, marright+IndWidtht);
        //setOuterDim(GraphWidthIND, GraphHeight);
        indicator.setPDims(dim[0], 0,IndWidtht,IndHeight);
      } else {
        setDim(dim[0],dim[1]);
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
      if (hasInd) parent.rect(0, -dim[1], dim[0], dim[1], 0, bord, bord, 0); //<>//
      else parent.rect(0, -dim[1], dim[0], dim[1], 0, 0, 0, 0);
      parent.popStyle();
    }

    void Draw() {
      beginDraw();
      drawBackground();
      drawBox();
      drawXAxis();
      drawYAxis();
      //plot.drawTopAxis();
      //plot.drawRightAxis();
      drawTitle();
      try{
        drawLines();
      }catch(IndexOutOfBoundsException e){
        e.printStackTrace();
      }catch(NullPointerException e){
        e.printStackTrace();
      }
      if (hasInd) indicator.draw();
      endDraw();
    }
  }

  class IND {
    float Height = 80;
    float Widtht = 80;
    float posX = 30;
    float posY = 30;
    final int bordRad = 5;
    protected final PApplet parent;
    
    String text = "95";

    final color fillc = color(255, 255, 255), lienc = color(0, 255, 0);
    float lineWidth = 1;
    IND(PApplet parent_,float [] siz) {
      Widtht = siz[1];
      Height = siz[1];
      parent = parent_;
    }

    void setPDims(float x, float y,float dimX, float dimY) {
      posX = x; 
      posY = y;
      Widtht = dimX;
      Height = dimY;
      //println(x,y);
    }
    
    void update(String txt){
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
      translate(posX+2,posY);
      parent.rect(0, -Height, Widtht, Height, bordRad, bordRad, bordRad, bordRad);
      drawText();
      popMatrix();
      parent.popStyle();
    }
    
    void drawText(){
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
