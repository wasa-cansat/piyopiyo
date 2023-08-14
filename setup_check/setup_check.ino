#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include <SPI.h>
#include "motor.h"
#include "HUSKYLENS.h"
#include <Adafruit_BMP280.h>
#include <Servo.h>

//servo
Servo myservo; 

// //HUSKYLENS
// HUSKYLENS huskylens;
// void findObject();
// void findObjectSetup();
// bool cam_sw = false;

//35.704978,139.713760
//GPS関連
TinyGPSPlus gps;
double destLatitude = 35.704978;    // 目的地の緯度
double destLongitude = 139.713760;  // 目的地の経度
int RX_PIN = 0;
int TX_PIN = 1;
SoftwareSerial gpsSerial(RX_PIN, TX_PIN);
struct LocationData {
  double latitude;
  double longitude;
  double bearing;
  double distance;
};
// void startTime();
LocationData getGPSData(double angle);

//bno055関連
double maxx;
double maxy;
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//モーター関連
Motor motor;

//SD関連
File myFile;
void sd_setup();
// const int chipSelect = 10;
void sd_GPSwrite(double latitude, double longitude, double bearing, double distance, double angle);

//落下検知
void sd_write(double height, double accel, double stat);
void fall_detect();

// 落下距離(m)の閾値[30.0 ±0.6]
const float fallDistance = 1.0;
// 自由落下時の加速度(m/s2)の閾値[0.5]
const float fallAccelThreshold = 0.0;
// 着地後静止時の加速度変化(m/s2)の誤差範囲(理論値は0)[0.1]
const float landedAccelThreshold = 0.1;
// 着地後静止時の気圧変化(Pa)の誤差範囲(理論値は0）[30]
const float landedPressureThreshold = 30;
// 着地後静止時の大まかな検出待機時間(自然数sec)[15]
const unsigned long landedIdleDuration = 1;
// 落下開始から強制分離までの時間(sec)[90]
const unsigned long ForcequitDuration = 5;

//BME280
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
Adafruit_BMP280 bmp; // I2C

//MNO055
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address

// 地表の気圧(kPa)
float basePressure = 101.325;
// 上昇高度の最高値を記録する変数
float highestAltitude = 0.0;
// ステータス定義
enum State {
  CHECK_FALLING,
  CHECK_LANDED,
  EXIT,
  COMPLETE
};
//最初のステータス
State state = CHECK_FALLING;
//　落下開始時刻を記録する変数
unsigned long fallStartTime;
// 前回の加速度を記録する変数
float lastAccel;
// 前回の気圧を記録する変数
float lastPressure;
//CHECK_LANDED内のfor文用
int i,j = 0;

unsigned long time, previous_time;

int n= 0;

void setup(void)
{
  Serial.begin(115200);
  Serial.println(n); n++;
  //気圧
  unsigned status;
  status = bmp.begin(0x76); 
  if (!status) { 
//        digitalWrite(LEDR, LOW);
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);

  }
  // 地表の気圧を読み取る
  basePressure = bmp.readPressure();
  lastPressure = basePressure;

  Serial.println(n); n++;
  //SD用
  // sd_setup();

  Serial.println(n); n++;
  //servo
  myservo.attach(6);
  myservo.write(180);
  delay(2000);
  myservo.write(90);
  delay(2000);
  myservo.write(160);
  
  Serial.println(n); n++;
  //モーター用
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println(n); n++;
  //GPS
  gpsSerial.begin(9600);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);

  Serial.println(n); n++;
  //bno055
  while (!Serial) {
    delay(10); 
    Serial.println("no Serial");
  } // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); //Serial.println("");

  Serial.println(n); n++;
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  Serial.println(n); n++;
  // //camera
  // Wire.begin();
  // while (!huskylens.begin(Wire))
  // {
  //   Serial.println(F("Begin failed!"));
  //   Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
  //   Serial.println(F("2.Please recheck the connection."));
  //   delay(100);
  Serial.println("no issues in setup");
}

void sd_setup(){
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    delay(1000);
    Serial.println("sd setup failed");
  }
}

void loop(){
  while(1){

  }
}
