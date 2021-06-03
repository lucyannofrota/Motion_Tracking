class offset{
  int value;
  offset(int v){ this.value = v;}
}

char readChar(byte[] data,offset offset){
  char ret = char(data[offset.value]);
  offset.value++;
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
  pos.X = readInt(data,offset);
  pos.Y = readInt(data,offset);
  pos.Z = readInt(data,offset);
  pos.Rx = readInt(data,offset);
  pos.Ry = readInt(data,offset);
  pos.Rz = readInt(data,offset);
}

byte[] COMBuff = new byte[64];

void serialEvent (Serial myPort) {
  IMU sensor = new IMU();
  //Implementar metodos para enviar a recebida pelo sensors
  int ac = millis(); 
  //println("Time:" + (ac-lm));
  lm = ac;
  int charCount = 6;
  offset of = new offset(0);
  if (myPort.available()>0) {
    byte c = byte(myPort.read());
    //println(c);
    if(c == '{'){
      c = byte(myPort.read());
      //println(c);
      switch(c){
        case 'P':
          COMBuff = myPort.readBytes(charCount * 2);
          readPose(COMBuff,of,CubePose);
          CubePose.setZero();
          break;
      }
    }
    else{
      print(c);
    }
  }
}
