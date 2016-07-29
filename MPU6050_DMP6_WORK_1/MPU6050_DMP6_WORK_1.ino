#include "I2Cdev.h"
#include <Bounce.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

//#define HELLO_WORLD
//#define FAST_MODE
//#define READ_MODE
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_WORLDACCEL

#define LED_PIN 13
bool blinkState = false;
#define BUTTON1 11
#define BUTTON2 12

Bounce button1 = Bounce(BUTTON1,5); 
Bounce button2 = Bounce(BUTTON2,5); 

int buttons[2];

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// orientation/motion vars
Quaternion q;
VectorInt16 aa;
VectorInt16 aaReal;
VectorInt16 aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    //Serial.begin(38400);
    Serial.begin(115200);
   
    while (!Serial);

    mpu.initialize();
    devStatus = mpu.dmpInitialize();
 
      // V2 - big plate
     mpu.setXGyroOffset(58);
    mpu.setYGyroOffset(-33);
    mpu.setZGyroOffset(12);

    mpu.setXAccelOffset(-1224);
    mpu.setYAccelOffset(-1095);
    mpu.setZAccelOffset(2204);


   
      // V2
  /*    mpu.setXGyroOffset(10);
    mpu.setYGyroOffset(-2);
    mpu.setZGyroOffset(37);
    mpu.setZAccelOffset(2606);
    */
/*
//V3
     mpu.setXGyroOffset(54);
    mpu.setYGyroOffset(-33);
    mpu.setZGyroOffset(14);
    mpu.setZAccelOffset(2191);
*/
    pinMode(BUTTON1,INPUT);
    pinMode(BUTTON2,INPUT);
   
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.println(F(")"));
    }
    pinMode(LED_PIN, OUTPUT);
}
short qw,qx,qy,qz;
           
void loop() {
    if (!dmpReady) return;

    while (!mpuInterrupt && fifoCount < packetSize) {}

    if ( button1.update() ) {
      if ( button1.read() == HIGH)
        buttons[0] = true;
      else
        buttons[0] = false;
    }

    if ( button2.update() ) {
      if ( button2.read() == HIGH)
        buttons[1] = true;
      else
        buttons[1] = false;
    }
    
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;



        #ifdef HELLO_WORLD
          Serial.println("Hello World");
          
        #endif

        
        #ifdef FAST_MODE
            mpu.dmpGetQuaternion(&q, fifoBuffer);
           qw=q.w*10000;
           qx=q.x*10000;
           qy=q.y*10000;
           qz=q.z*10000;
           qw=qw*2;
           qx=qx*2;
           qy=qy*2;
           qz=qz*2;
           byte but = buttons[0]*128+buttons[1]*64+63;
           byte StopByte = 255;
          byte bytearray[10] = {StopByte,but,lowByte(qx),highByte(qx),lowByte(qy),highByte(qy),lowByte(qz),highByte(qz),lowByte(qw),highByte(qw)};
          Serial.write(bytearray,10);
        #endif

         #ifdef READ_MODE
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetEuler(euler, &q);
              Serial.println("Hello! "+String(euler[0] * 180/M_PI) + "," + String(euler[1] * 180/M_PI) + "," + String(euler[2] * 180/M_PI));
        #endif

        #ifdef OUTPUT_READABLE_QUATERNION
            mpu.dmpGetQuaternion(&q, fifoBuffer);
           short qw,qx,qy,qz;
           qw=q.w*10000;
           qx=q.x*10000;
           qy=q.y*10000;
           qz=q.z*10000;
           qw=qw*2;
           qx=qx*2;
           qy=qy*2;
           qz=qz*2;
           byte but = buttons[0]*128+buttons[1]*64+63;
           byte StopByte = 255;
           
          byte bytearray[10] = {StopByte,but,lowByte(qx),highByte(qx),lowByte(qy),highByte(qy),lowByte(qz),highByte(qz),lowByte(qw),highByte(qw)};
        if (buttons[1]==0)
         {
          Serial.write(bytearray,10);
         }
         else 
          {
               mpu.dmpGetEuler(euler, &q);
              Serial.println("Hello! "+String(euler[0] * 180/M_PI) + "," + String(euler[1] * 180/M_PI) + "," + String(euler[2] * 180/M_PI));
       //      Serial.println("Hello! " + String(qx) + "," + String(qy) + "," + String(qz) +","+String(qw));
          
          }
        //   Serial.println(String(qw) + "," + String(qx) + "," + String(qy) + "," + String(qz) + "," + String(but) + "," +","+String(millis()));
        //    Serial.println(String(q.w,4) + "," + String(q.x,4) + "," + String(q.y,4) + "," + String(q.z,4) + "," + String(buttons[0]) + "," + String(buttons[1])+","+String(millis()));

        #endif

        #ifdef OUTPUT_READABLE_EULER
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.println(String(euler[0] * 180/M_PI) + "," + String(euler[1] * 180/M_PI) + "," + String(euler[2] * 180/M_PI));
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aa.x);
            Serial.print("\t");
            Serial.print(aa.y);
            Serial.print("\t");
            Serial.print(aa.z);
            Serial.print("\t");
            Serial.print("\t");
            Serial.print(gravity.x);
            Serial.print("\t");
            Serial.print(gravity.y);
            Serial.print("\t");
            Serial.println(gravity.z);
        #endif


        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
