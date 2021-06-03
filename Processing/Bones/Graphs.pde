public class myGraphWindow extends PApplet{

  int time = 0; //time in ms
  
  GPlot mainPlot;
  GPlot mainPlot2;
  
  
  //PlotGroupIMU compPlot;
  PlotGroupIMU Sensor1;
  PlotGroupIMU Sensor2;
  PlotGroupIMU Sensor3;
  PlotGroupIMU Sensor4;
  
  boolean flag;
  
  myGraphWindow(){
    super();
    PApplet.runSketch(new String[]{this.getClass().getName()}, this);
  }
  
  public void settings(){
    //size(1920, 1080);
    fullScreen();
    flag = false;
  }
  
  public void setup(){
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
    for(int i = 0; i < 1000; i++){
      points1.add(new GPoint(i,pow(i,1.5)));
      points2.add(new GPoint(i,pow(1000-i,1.5)));
    }
    mainPlot.setPoints(points1);
    mainPlot2.setPoints(points2);
    GPointsArray temp1 = mainPlot.getMainLayer().getPointsRef();
    println(temp1.getLastPoint().getY());
    GPointsArray temp2 = mainPlot2.getMainLayer().getPointsRef();
    for(int i = 1001; i < 2000; i++){
      temp1.add(new GPoint(i,pow(i,1.5)));
      temp2.add(new GPoint(i,pow(2000-i,1.5)));
    }
    println(mainPlot.getMainLayer().getPointsRef().getLastPoint().getY());
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
  
  void mouseClicked(){
    flag = !flag;
  }
  
  void newData(IMU sens,int nS){
    int elt;
    if(time == 0){
      time = millis();
      elt = 0;
    }
    else{
      elt = millis() - time;
    }
    switch(nS){
      case 1:
        Sensor1.addPoint(elt,sens);
      break;
      case 2:
        Sensor2.addPoint(elt,sens);
      break;
      case 3:
        Sensor3.addPoint(elt,sens);
      break;
      case 4:
        Sensor4.addPoint(elt,sens);
      break;
    }
  }
  
  
  class PlotGroupIMU{
    GPlot ax, ay, az;
    GPlot gx, gy, gz;
    
    PlotGroupIMU(PApplet parent){
      final int plotSpace = 250;
      final int plotIntSpace = 0;
      ax = new GPlot(parent);
      //setAllFontProperties(String fontName, int fontColor, int fontSize) 
      ax.setTitleText("X Acceleration");
      ax.getTitle().setOffset(1);
      ax.setPos(165, 5);
      ax.setMar(4,30,15,2);
      ax.setDim(1720, 150);
      //setOffset
      ay = new GPlot(parent);
      ay.setTitleText("Y Acceleration");
      ay.setPos(165, 5+plotSpace+plotIntSpace);
      ay.setDim(1650, 150);
      //az = new GPlot(parent);
      //az.setPos(165, 5+plotSpace+10+plotSpace+10);
      //az.setDim(1650, 150);
      
      //gx = new GPlot(parent);
      //gx.setPos(165, 5+plotSpace+10+plotSpace+10+plotSpace+10);
      //gx.setDim(1650, 150);
      //gy = new GPlot(parent);
      //gy.setPos(165, 5+plotSpace+10+plotSpace+10+plotSpace+10+plotSpace+10);
      //gy.setDim(1650, 150);
      //gz = new GPlot(parent);
      //gz.setPos(165, 5+plotSpace+10+plotSpace+10+plotSpace+10+plotSpace+10+plotSpace+5);
      //gz.setDim(1650, 150);
    }
    
    void addPoint(int time,IMU sens){
      ax.addPoint(new GPoint(time,sens.ax));
      ay.addPoint(new GPoint(time,sens.ay));
      az.addPoint(new GPoint(time,sens.az));
      gx.addPoint(new GPoint(time,sens.gx));
      gy.addPoint(new GPoint(time,sens.gy));
      gz.addPoint(new GPoint(time,sens.gz));
    }
    
    
    void draw(){
        drawPlot(ax);
        drawPlot(ay);
        //drawPlot(az);
        //drawPlot(gx);
        //drawPlot(gy);
        //drawPlot(gz);
    }
    
    void drawPlot(GPlot plot){
        plot.beginDraw();
        plot.drawBackground();
        plot.drawBox();
        //plot.drawXAxis();
        plot.drawYAxis();
        //plot.drawTopAxis();
        //plot.drawRightAxis();
        plot.drawTitle();
        plot.getMainLayer().drawPoints();
        plot.endDraw();
    }
  }
}
