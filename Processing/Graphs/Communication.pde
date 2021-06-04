class offset{
  int value;
  offset(int v){ this.value = v;}
}

char readChar(byte[] data,offset offset){
  char ret = char(data[offset.value]);
  offset.value++;
  return ret;
}

int readShort(byte[] data,offset offset){
  int ret;
  if((data[offset.value] & 0x80) == 0x80){//Negative
    ret = (data[offset.value] & 0x7f) | 
          ((0x00) << 8) | 
          (0xffffff80);
  }
  else{//Positive
    ret = (data[offset.value] & 0xff) | 
          ((0x00) << 8);
  }
  offset.value+=1;
  return ret;
}

int readInt(byte[] data,offset offset){
  int ret;
  if((data[offset.value+1] & 0x80) == 0x80){//Negative
    ret = (data[offset.value] & 0xff) | 
          ((data[offset.value+1] & 0x7f) << 8) | 
          (0xffff8000);
  }
  else{//Positive
    ret = (data[offset.value] & 0xff) | 
          ((data[offset.value+1] & 0xff) << 8);
  }
  offset.value+=2;
  return ret;
}

void readPose(byte[] data,offset offset,Pose pos){
  int a = readShort(data,offset);
  //print("Index: ");
  //println(a);
  pos.X = readInt(data,offset);
  pos.Y = readInt(data,offset);
  pos.Z = readInt(data,offset);
  pos.Rx = readInt(data,offset);
  pos.Ry = readInt(data,offset);
  pos.Rz = readInt(data,offset);
}

int readSensor(byte[] data,offset offset,IMU sens){
  int a = readShort(data,offset);
  //print("Index: ");
  //println(a);
  sens.ax = readInt(data,offset);
  sens.ay = readInt(data,offset);
  sens.az = readInt(data,offset);
  sens.rx = readInt(data,offset);
  sens.ry = readInt(data,offset);
  sens.rz = readInt(data,offset);
  return a;
}

byte[] COMBuff = new byte[64];


void serialEvent (Serial myPort) {
  IMU sensor = new IMU();
  int ac = millis(); 
   delay(10);
  final int maxbuf = buffReadUntil;
  offset of = new offset(0);
  int len = myPort.available();
  
  if (len > 0) {
    if(len < maxbuf) delay(10);
    COMBuff = myPort.readBytes(1);
    //println(c);
    if(COMBuff[0] == '{'){
      COMBuff = myPort.readBytes(1);
      //println(c);
      switch(COMBuff[0]){
        case 'P':
          COMBuff = myPort.readBytes(6 * 2 + 1);
          //readPose(COMBuff,of,CubePose);
          //CubePose.print();
          println("Time:" + (ac-lm));
          lm = ac;
          break;
        case 'S':
          COMBuff = myPort.readBytes(6 * 2 + 1);
          //println(COMBuff);
          int b = readSensor(COMBuff,of,sensor);
          //println(b);
          GraphsH.newData(sensor,b);
          //CubePose.print();
          println("Time:" + (ac-lm));
          lm = ac;
          break;
      }
    }
    
  }
}
