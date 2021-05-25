// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
# define M_PI           3.14159265358979323846


class MyMPU6050 {
    
    public:
    
        MyMPU6050(PinName i2cSda, PinName i2cScl, PinName interrupt) : mpu(i2cSda, i2cScl), checkpin(interrupt){
            resetTheta();
        }
        
//        void setPC(Serial *pc){
//            _pc = pc;
//        }
        
        void loop() {
            // if programming failed, don't try to do anything
            if (!dmpReady) return;
            
            printf("dmp ready\n");
            
            // wait for MPU interrupt or extra packet(s) available
            if (!mpuInterrupt && fifoCount < packetSize) {
                return;
                // other program behavior stuff here
                // .
                // .
                // .
                // if you are really paranoid you can frequently test in between other
                // stuff to see if mpuInterrupt is true, and if so, "break;" from the
                // while() loop to immediately process the MPU data
                // .
                // .
                // .
            }
            
            printf("mpu interrupt \n");
                                
            // reset interrupt flag and get INT_STATUS byte
            mpuInterrupt = false;
            mpuIntStatus = mpu.getIntStatus();
        
            // get current FIFO count
            fifoCount = mpu.getFIFOCount();
        
            // check for overflow (this should never happen unless our code is too inefficient)
            if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                // reset so we can continue cleanly
                mpu.resetFIFO();
                printf("FIFO overflow!\n");
        
            // otherwise, check for DMP data ready interrupt (this should happen frequently)
            } else if (mpuIntStatus & 0x02) {
                // wait for correct available data length, should be a VERY short wait
                while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
                // read a packet from FIFO
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                
                // track FIFO count here in case there is > 1 packet available
                // (this lets us immediately read more without waiting for an interrupt)
                fifoCount -= packetSize;
        
                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                float newTheta;
                if (ypr[1]>0) newTheta = ypr[2];
                else if (ypr[2]<0) newTheta = -M_PI-ypr[2];
                else newTheta = M_PI-ypr[2];
                
                mpu.dmpGetGyro(gyroData, fifoBuffer);
                float newDTheta = -gyroData[2]/180.0*M_PI;
                
                if (!thetaInitFlag && abs(newTheta-theta)>0.3) {
                    printf("IMU interrupt clash\n");
                    mpu.resetFIFO();
                } else {
                    thetaInitFlag = false;
                    theta = newTheta;
                    dtheta = newDTheta;
//                    printf("loop %f\n", theta);
                }
                                
                                
                
            }
        }
        
        void dmpDataReady() {
            mpuInterrupt = true;
        }
        
        float getTheta(){
//            _pc->printf("%f\n", theta);
            return theta;
        }
        
        float getDTheta(){
            return dtheta;
        }
        
        void disable(){
            dmpReady = false;            
            mpuInterrupt = false;
            checkpin.rise(NULL);
        }
        
        void disableInterrupt(){
            mpuInterrupt = false;
            checkpin.rise(NULL);
        }
        
        void enable(){
            setup();
        }
        
        void enableInterrupt(){
            checkpin.rise(this, &MyMPU6050::dmpDataReady);
        }

    private:
    
//        Serial *_pc;
    
        MPU6050 mpu;
        
        bool dmpReady;  // set true if DMP init was successful
        uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
        uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
        uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
        uint16_t fifoCount;     // count of all bytes currently in FIFO
        uint8_t fifoBuffer[64]; // FIFO storage buffer
        
        // orientation/motion vars
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorFloat gravity;    // [x, y, z]            gravity vector
        float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
        int16_t gyroData[3];
        volatile float theta;
        bool thetaInitFlag;
        volatile float dtheta;
    
        InterruptIn checkpin;
        volatile bool mpuInterrupt;
        
        
        void resetTheta(){
            theta = 0;
            dtheta = 0;
            thetaInitFlag = true;
        }
        
        void setup() {
            
            resetTheta();
            
            dmpReady = false;            
            mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
        
            // initialize device
            printf("Initializing I2C devices...\r\n");
            mpu.initialize();
        
            // verify connection
            printf("Testing device connections...\r\n");
            mpu.testConnection();//if() pc.printf("MPU6050 connection successful\r\n");
//            else pc.printf("MPU6050 connection failed\r\n");
        
            // load and configure the DMP
            printf("Initializing DMP...\r\n");
            devStatus = mpu.dmpInitialize();
            
            // make sure it worked (returns 0 if so)
            if (devStatus == 0) {
                // turn on the DMP, now that it's ready
                printf("Enabling DMP...\r\n");
                mpu.setDMPEnabled(true);
        
                // enable Arduino interrupt detection
                printf("Enabling interrupt detection (Arduino external interrupt 0)...\r\n");
                checkpin.rise(this, &MyMPU6050::dmpDataReady);
                
                mpuIntStatus = mpu.getIntStatus();
        
                // set our DMP Ready flag so the main loop() function knows it's okay to use it
                printf("DMP ready! Waiting for first interrupt...\r\n");
                dmpReady = true;
        
                // get expected DMP packet size for later comparison
                packetSize = mpu.dmpGetFIFOPacketSize();
            } else {
                // ERROR!
                // 1 = initial memory load failed
                // 2 = DMP configuration updates failed
                // (if it's going to break, usually the code will be 1)
                
                printf("DDMP Initialization failed (code ");
                printf("%d", devStatus);
                printf(")\r\n");
            }
        
        }
};
