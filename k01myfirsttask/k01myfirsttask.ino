#include <krnl.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"
#include <Wire.h>

struct k_t *p1, *p2, *p3, *p4, *p5, *dk1, *dk2, *dk3;
struct k_t *sensor1, *sensor2, *sensor3, *sender;
float d1, d2, d3;

MPU6050 mpu;

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

unsigned char data[4] = {};
int sum;
void t1()
{
  k_set_sem_timer(sensor1, 20);
  while (1) {
    k_wait(sensor1, 0);
    if (Serial1.read() == 0xff)
    {
      data[0] = 0xff;
      for (int i = 0; i < 4; i++)
      {
        data[i] = Serial1.read();
      }
      Serial1.flush();
      sum = (data[0] + data[1] + data[2]) & 0x00FF;
      if (sum == data[3])
      {
        k_wait(dk1, 0);
        d1 = (data[1] << 8) + data[2];
        k_signal(dk1);
      }
    }
  }
}

unsigned char data2[4] = {};
int sum2;
void t2()
{
  k_set_sem_timer(sensor2, 20);
  while (1) {
    k_wait(sensor2, 0);
    if (Serial2.read() == 0xff)
    {
      data2[0] = 0xff;
      for (int i = 1; i < 4; i++)
      {
        data2[i] = Serial2.read();
      }

      Serial2.flush();

      if (data2[0] == 0xff)
      {
        sum2 = (data2[0] + data2[1] + data2[2]) & 0x00FF;
        if (sum2 == data2[3])
        {
          k_wait(dk2, 0);
          d2 = (data2[1] << 8) + data[2];
          k_signal(dk2);
        }
      }
    }
  }
}

unsigned char data3[4] = {};
int sum3;
void t3()
{
  k_set_sem_timer(sensor3, 20);
  while (1) {
    k_wait(sensor3, 0);
    if (Serial3.read() == 0xff)
    {
      data3[0] = 0xff;
      for (int i = 1; i < 4; i++)
      {
        data3[i] = Serial3.read();
      }
      Serial3.flush();

      sum3 = (data3[0] + data3[1] + data3[2]) & 0x00FF;
      if (sum3 == data3[3])
      {
        k_wait(dk3, 0);
        d3 = (data3[1] << 8) + data3[2];
        k_signal(dk3);
      }

    }
  }
}

unsigned int delay_time = 0;
void t4()
{
  k_set_sem_timer(sender, 100);

  digitalWrite(8, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(10, HIGH);
  while (1) {
    k_wait(sender, 0);

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
      
      // display Euler angles in degrees
      
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  
      k_wait(dk1, 0);
      k_wait(dk2, 0);
      k_wait(dk3, 0);
  
      Serial.write(0xff);
  
      byte * b6 = (byte *) &d1;
      Serial.write(b6, 4);
      k_signal(dk1);
  
      //Serial.write(0x20);
      byte * b4 = (byte *) &d2;
      Serial.write(b4, 4);
      k_signal(dk2);
  
      //Serial.write(0x20);
      byte * b5 = (byte *) &d3;
      Serial.write(b5, 4);
      k_signal(dk3);
  
      //Serial.write(0x20);
      byte * b = (byte *) &ypr[2];
      Serial.write(b, 4);
      
      //Serial.write(0x20);
      byte * b2 = (byte *) &ypr[1];
      Serial.write(b2, 4);
      
      //Serial.write(0x20);
      byte * b3 = (byte *) &ypr[0];
      Serial.write(b3, 4);
    }
  }
}


void setup()
{
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Serial.begin(115200);
  
  mpu.initialize();

  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    mpu.getIntStatus();
    mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);


  int res;
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);

  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);

  digitalWrite(8, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);

  Serial.print("Begin");

  k_init(4, 12, 0); // init with space for one task

  sensor1 = k_crt_sem(0, 1);
  sensor2 = k_crt_sem(0, 1);
  sensor3 = k_crt_sem(0, 1);
  sender = k_crt_sem(0, 1);

  dk1 = k_crt_sem(1, 1);
  dk2 = k_crt_sem(1, 1);
  dk3 = k_crt_sem(1, 1);

  // priority low number higher priority than higher number
  p1 = k_crt_task(t1, 2, 100); // t1 as task, priority 10, 100 B stak
  p2 = k_crt_task(t2, 3, 100); // t1 as task, priority 10, 100 B stak
  p3 = k_crt_task(t3, 4, 100); // t1 as task, priority 10, 100 B stak
  p4 = k_crt_task(t4, 1, 1000); // t1 as task, priority 10, 100 B stak

  res = k_start(1); // 1 milli sec tick speed
  // you will never return from k_start
  Serial.print("ups an error occured: "); Serial.println(res);
  while (1) ;
}

void loop() {
}
