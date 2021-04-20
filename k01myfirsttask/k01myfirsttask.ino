#include <krnl.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

struct k_t *p1, *p2, *p3, *p4, *p5, *dk1, *dk2, *dk3;
struct k_t *sensor1, *sensor2, *sensor3, *sender;
float d1, d2, d3;

Adafruit_MPU6050 mpu;

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
    
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

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
    byte * b = (byte *) &g.gyro.x;
    Serial.write(b, 4);
    
    //Serial.write(0x20);
    byte * b2 = (byte *) &g.gyro.y;
    Serial.write(b2, 4);
    
    //Serial.write(0x20);
    byte * b3 = (byte *) &g.gyro.z;
    Serial.write(b3, 4);

  }
}


void setup()
{
  Serial.begin(115200);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);


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

  res = k_start(10); // 1 milli sec tick speed
  // you will never return from k_start
  Serial.print("ups an error occured: "); Serial.println(res);
  while (1) ;
}

void loop() {
}
