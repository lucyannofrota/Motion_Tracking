class Plane {
  float x,y;
  float size;
  float scale;
  
  Plane(float posx, float posy, float s){
    x = posx;
    y = posy;
    scale = s;
  }
  void display(){
    //pushMatrix();
    scale(1,1,1.2);
    // body
    pushMatrix();
    fill(#5AE5CB);
    scale(0.6,0.5,1.3);
    sphere(1); 
    
    popMatrix();
    
    // ball
    pushMatrix();
    scale(0.5);
    fill(0,100,200);
    translate(0.0,-0.35,0.45);
    sphere(1);
    popMatrix();
    
    //wings
    pushMatrix();
    fill(100,0,100);
    translate(0,0.1,0.25);
    scale(1.6,0.2,0.5);
    sphere(1); 
    popMatrix();
    
    //coiso para cima
    pushMatrix();
    fill(100,0,100);
    translate(0,-0.25,-1);
    scale(0.1,0.4,0.3);
    sphere(1); 
    popMatrix();
    
    //coiso para os lados
    pushMatrix();
    fill(100,0,100);
    translate(0,0,-0.95);
    scale(0.8,0.22,0.35);
    sphere(1); 
    popMatrix();
    
    
  }
  
}
