#include "I2Cdev.h"
#include <Bounce.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

//#define PLATE_1  // FIRST GYRO + BIG PLATE
//#define PLATE_2 // ACCURATE GYRO 
//#define PLATE_3 // LAST GYRO with #2 +  CONTROLLER
#define PLATE_4

//#define HELLO_WORLD
//#define FAST_MODE
#define FAST_MODE_ACCEL
//#define TIMER_MODE
//#define READ_MODE
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
//#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_WORLDACCEL

#define LED_PIN 13
bool blinkState = false;
#define BUTTON1 11
#define BUTTON2 12
#define BUTTON3 10

Bounce button1 = Bounce(BUTTON1,5); 
Bounce button2 = Bounce(BUTTON2,5); 
Bounce button3 = Bounce(BUTTON3,5); 

int buttons[3];
#ifdef TIMER_MODE
  unsigned long timer1;
  byte buf[4];  
#endif
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
 
   #ifdef PLATE_1
      Serial.println("Plate 1");
      mpu.setXGyroOffset(58);
      mpu.setYGyroOffset(-33);
      mpu.setZGyroOffset(12);
   // mpu.setXAccelOffset(-1224);
  //  mpu.setYAccelOffset(-1095);
  //  mpu.setZAccelOffset(2204);
      mpu.setZAccelOffset(1334);
  #endif
//   -387 -3354 1645  14  -4  37

   #ifdef PLATE_2
   
      Serial.println("Plate 2");
      mpu.setXGyroOffset(14);
      mpu.setYGyroOffset(-4);
      mpu.setZGyroOffset(37);
      mpu.setXAccelOffset(-387);
      mpu.setYAccelOffset(-3354);
      mpu.setZAccelOffset(1645);
  #endif
  //1845  1501  247 71  -11 41
//1956  1561  241 67  -9  49
 #ifdef PLATE_3
 
      Serial.println("Plate 3");
      mpu.setXGyroOffset(71);
      mpu.setYGyroOffset(-11);
      mpu.setZGyroOffset(41);
      mpu.setXAccelOffset(1845);
      mpu.setYAccelOffset(1501);
      mpu.setZAccelOffset(1400);
  #endif

   #ifdef PLATE_4
 
      Serial.println("Plate 4");
      mpu.setXGyroOffset(17);
      mpu.setYGyroOffset(-2);
      mpu.setZGyroOffset(-30);
      mpu.setXAccelOffset(-1790);
      mpu.setYAccelOffset(240);
      mpu.setZAccelOffset(1445);
  #endif

      // V2
      //-1753  72  1454  15  -1  -32
//Your offsets:  -1789 240 1442  17  -3  -30
//-1790  240 1445  17  -2  -30
//-1659  261 1461  18  -2  -30

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
    pinMode(BUTTON3,INPUT);
    
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
        pinMode(10, OUTPUT);

}
short qw,qx,qy,qz,ax,ay,az;
           
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
    
    if ( button3.update() ) {
      if ( button3.read() == HIGH)
        buttons[2] = true;
      else
        buttons[2] = false;
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


        #ifdef TIMER_MODE
          timer1 =millis();
          buf[0] = (byte) timer1;
`          buf[1] = (byte) (timer1 >> 8);
          buf[2] = (byte) (timer1 >> 16);
          buf[3] = (byte) (timer1 >> 24);
           byte but = buttons[0]*128+buttons[1]*64+63;
           byte StopByte = 255;
          byte bytearray[10] = {StopByte,but,buf[0],buf[1],buf[2],buf[3],buf[3],buf[2],buf[1],buf[0]};
          Serial.write(bytearray,10);    
         // Serial.println("Hello "+String(buf[1])+" "+String(buf[0]));
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

        
        #ifdef FAST_MODE_ACCEL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
           qw=q.w*10000;
           qx=q.x*10000;
           qy=q.y*10000;
           qz=q.z*10000;
           qw=qw*2;
           qx=qx*2;
           qy=qy*2;
           qz=qz*2;
           #ifdef PLATE_2
           byte but = buttons[1]*128 +buttons[0]*64+buttons[2]*32+ 31;
           #endif
           #ifdef PLATE_1
           byte but = buttons[1]*128 +buttons[0]*64+buttons[2]*32+ 31;
           #endif
           #ifdef PLATE_3
           byte but = buttons[0]*128 +buttons[1]*64+buttons[2]*32+ 31;
           #endif
          #ifdef PLATE_4
           byte but = buttons[1]*128 +buttons[0]*64+buttons[2]*32+ 31;
           #endif


           byte StopByte = 255;
           mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
          
             ax = aaWorld.x;
             ay = aaWorld.y;
             az = aaWorld.z;
          byte bytearray[16] = {StopByte,but,lowByte(qx),highByte(qx),lowByte(qy),highByte(qy),lowByte(qz),highByte(qz),lowByte(qw),highByte(qw),lowByte(ax),highByte(ax),lowByte(ay),highByte(ay),lowByte(az),highByte(az)};
          Serial.write(bytearray,16);
        #endif

         #ifdef READ_MODE
              mpu.dmpGetQuaternion(&q, fifoBuffer);
              mpu.dmpGetEuler(euler, &q);
              
               mpu.dmpGetAccel(&aa, fifoBuffer);
           mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);          
             ax = aaWorld.x;
             ay = aaWorld.y;
             az = aaWorld.z;
              Serial.println("Hello! "+String(euler[0] * 180/M_PI) + "," + String(euler[1] * 180/M_PI) + "," + String(euler[2] * 180/M_PI)+" ac ="+String(ax)+" "+String(ay)+" "+String(az)+" B="+String(buttons[0])+String(buttons[1])+String(buttons[2]));
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
