#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include "motor.h"
#include "HUSKYLENS.h"
#include <Adafruit_BMP280.h>
#include <SD.h>


#include <Servo.h>


//servo
Servo myservo; 



//HUSKYLENS
HUSKYLENS huskylens;
void findObject();
void findObjectSetup();
bool cam_sw = false;

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

// void startTime();p
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
 const int chipSelect = 10;
void sd_GPSwrite(double latitude, double longitude, double bearing, double distance, double angle, int motor_status);
//
////落下検知
//void sd_write(double height, double accel, double stat){
//  if (myFile){
//
//      myFile.print("height");
//      myFile.println(height);
//      myFile.print("accel");
//      myFile.println(accel);
//      myFile.print("status");
//      myFile.println(stat);
//    }
//  else{
//    Serial.println("cannot write to file");
//  }
//}

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


unsigned long time, previous_time, sd_time, sd_previous_time;



void setup(void) 
{

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
  

  



  //モーター用
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  //GPS
  Serial.begin(115200);
  gpsSerial.begin(9600);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);

    //SD用
  sd_setup();

  //servo
  myservo.attach(6);
  myservo.write(180);
  delay(10000);
  myservo.write(90);
  delay(10000);
  myservo.write(160);

  
  //bno055
  while (!Serial) {
    delay(10); 
    Serial.println("no Serial");
  } // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  //camera
  Wire.begin();
  while (!huskylens.begin(Wire))
 {
       Serial.println(F("Begin failed!"));
       Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
       Serial.println(F("2.Please recheck the connection."));
       delay(100);
 }

 

}


void loop(void)
{


  fall_detect();
  
  myservo.write(40);
  delay(2000);
  myservo.write(180);
  delay(1000);



  Serial.println("now in loop");
  double cal_x = 0;
  double error = 0;
  int motor_status = 0;
//  bool camera_dead = false;
  LocationData now_data = {0.0, 0.0, 0.0, 20.0};

  time = millis();
  sd_time = millis();

  
  while(1){
    sd_previous_time = millis();
    if(sd_previous_time - sd_time > 2000){
      motor.freeze(0);
      Serial.println("sd write");
      sd_GPSwrite(now_data.latitude, now_data.longitude, now_data.bearing, now_data.distance, cal_x, motor_status);
      sd_time = millis();
    }

    
    if (huskylens.request()) {
//      camera_dead = false;
      if (huskylens.isLearned()) {
          if (huskylens.available()) {
            findObject();
            continue;
          }
          else if(cam_sw){
            findObjectSetup();
          }
      }
      else{
//        camera_dead = true;
      }
    }
    else{
//      camera_dead = true;
    }

    

    now_data = getGPSData(cal_x);
//    if(camera_dead){
//      if(now_data.distance < 2.0){
//        motor.freeze(0);
//        while(1){
//          delay(10000);
//        }
//      }
//    }
    
    previous_time = millis();

    
    if(previous_time -time > 100){
      Serial.println(now_data.bearing);




    sensors_event_t orientationData, magnetometerData;

    bno.getEvent(&orientationData);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
    double x = orientationData.orientation.x;
    double y = orientationData.orientation.y;
    double z = orientationData.orientation.z;
    double mag_x = magnetometerData.magnetic.x;
    double mag_y = magnetometerData.magnetic.y;
    double mag_z = magnetometerData.magnetic.z;
    if(abs(mag_y + 0.06) < 0.0001 || abs(mag_y + 8.06) < 0.0001 || (mag_x + mag_y + mag_z) > 2000){
        // Serial.println("not valid");
    }
    else{
          error = atan2(mag_y, mag_x)*180/PI + 180 - x;
//        Serial.print("mag_x = ");
//        Serial.println(atan2(mag_y, mag_x)*180/PI + 180);
//        Serial.print("error: ");
//        Serial.println(error);
    } 
    cal_x = (x + error)<360 ? x + error: x + error - 360.0;
    cal_x = (cal_x + 180) < 360? cal_x + 180: cal_x - 180; //I2cテスト用にコメントアウト
   cal_x = 360 - cal_x;
    Serial.print("x = ");
    Serial.println(cal_x); 
    time = millis();     
    }


//    Serial.println(data.bearing);
    if(abs(cal_x - now_data.bearing) < 10.0)
    {
      motor.forward(0);
      motor_status = 1;
//      Serial.println("stop");
    }
    else if(((now_data.bearing - cal_x) > 0 ? now_data.bearing - cal_x: now_data.bearing + 360 - cal_x) < 180 )
    {
      motor.go_right(0);
//      Serial.println("right");
      motor_status = 3;
    }
    else
    {
      motor.go_left(0);
//      Serial.println("left");
      motor_status = 4;
    }
  }
}


//GPS関連
// void startTime() {
//   unsigned long startTime = millis();
//   Serial.print(String(startTime / 1000) + "秒　　　");
// }

LocationData getGPSData(double angle) {
  static LocationData data = { 0.0, 0.0, 0.0, 20.0 };

  while (gpsSerial.available() > 0) {

    char c = gpsSerial.read();
    gps.encode(c);

    if (gps.location.isUpdated()) {
      
      data.latitude = gps.location.lat();
      data.longitude = gps.location.lng();

      data.bearing = TinyGPSPlus::courseTo(
        data.latitude, data.longitude,
        destLatitude, destLongitude);
      data.distance = TinyGPSPlus::distanceBetween(
        data.latitude, data.longitude,
        destLatitude, destLongitude);
      
//      unsigned long startTime = millis();
//      Serial.println(String(startTime / 1000) + "秒　　　");
      if(data.latitude != 0.0){
        Serial.print("現在地（");
        Serial.print(data.latitude, 6);
        Serial.print(",");
        Serial.print(data.longitude, 6);
        Serial.print(") ");
    
        Serial.print("方位: ");
        Serial.print(data.bearing, 6);
        Serial.print("°  ");
        Serial.print("距離: ");
        Serial.print(data.distance, 6);
        Serial.println("m");
        return data;
      }
    }
  }
}

////SD用

void sd_setup(){
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    delay(1000);
    Serial.println("sd setup failed");
  }
  myFile = SD.open("LOG.txt", FILE_WRITE);
  myFile.println("start log");
  myFile.close();
}

void sd_GPSwrite(double latitude, double longitude, double bearing, double distance, double angle, int motor_status){
  myFile = SD.open("LOG.txt", FILE_WRITE);
  if (myFile){
    myFile.print("time: ");
    unsigned long gps_time = millis();
    myFile.println(String(gps_time/1000));
      myFile.print(" Latitude: ");
      myFile.print(String(latitude,9));
      Serial.println(latitude, 9);
      myFile.print(" longitude: ");
      myFile.print(String(longitude,9));
      Serial.println(longitude, 9);
      myFile.print(" bearing: ");
      myFile.print(String(bearing,9));
      Serial.println(bearing, 9);
      myFile.print(" distance: ");
      myFile.println(String(distance,9));
      Serial.println(distance, 9);
      myFile.print(" angle: ");
      myFile.println(String(angle,9));
      Serial.println(angle, 9);
      myFile.print(" motor_status: ");
      myFile.println(String(motor_status,9));
      Serial.println(motor_status, 9);
    
  }
  else{
    myFile.close();
    motor.freeze(0);
    Serial.println("cannot open file");
    pinMode(10, OUTPUT);
    for(int i = 0; i < 3; i++){
      if(SD.begin(10)){
        
      }
      Serial.println("sd setup failed");
      delay(1000);
    }
  }
  myFile.close();
}


//huskylens

void findObject(){
  cam_sw = true;
              huskylens.saveScreenshotToSDCard();
              int n = huskylens.count();
              int i = 0;
              HUSKYLENSResult result_list[n];
              while (huskylens.available()) {
                  result_list[i] = huskylens.read();
                  i++;
              }
              int left = 320;
              int right = 0;
              for (int j = 0; j < n; j++) {
                  left = min(left, result_list[j].xCenter - (result_list[j].width / 2));
                  right = max(right, result_list[j].xCenter + (result_list[j].width / 2));
              }
              if (right - left > 100) {
                  Serial.println("husky_stop");
                  while (1) {
                    motor.freeze(2000);
                  }
              }
              if (left < 70) {
                  motor.go_left(100);
                  Serial.println("husky_left");
                  motor.freeze(0);
              } else if (right > 250) {
                  motor.go_right(100);
                  Serial.println("husky_right");
                  motor.freeze(0);
              } else {
                  motor.forward(1000);
                  Serial.println("husky_forward");
                  motor.freeze(0);
              }
              delay(100);
}





void findObjectSetup(){
  unsigned long cam_time, cam_previous_time;
  cam_previous_time = millis();
  cam_time = millis();
  while((cam_time - cam_previous_time) < 500){
    cam_time = millis();
          if (huskylens.available()) {
            findObject();
            return;
          }

  }

  //見つからないときの処理
  cam_sw = false;
  
 
}


void fall_detect(){
  int count = -1;
  double data[200][3];
  while(1){
    Serial.println(count);
    count++;
  
  // if(count >= 10){
  //   Serial.println("wrote in SD");
  //   delay(1000);

  //   myFile = SD.open("test.txt", FILE_WRITE);
  //   for(int i = 0; i < 10; i++){

  //     sd_write(data[i][0], data[i][1], data[i][2]);
  //     delay(20);
  //   }
  //   myFile.close();
  //   count = 0;

  // }

  
  Serial.println("in Loop");
  float pressure = bmp.readPressure();
  float altitude = 44330 * ( 1 - pow(pressure/basePressure, 1/5.255) );

  float x, y, z, accel;
  imu::Vector<3> accelermetor = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  x = accelermetor.x();
  y = accelermetor.y();
  z = accelermetor.z();
  accel = sqrt(x*x + y*y + z*z);
  Serial.println(accel);

  // turn all the LEDs off
//  digitalWrite(LEDR, HIGH);
//  digitalWrite(LEDG, HIGH);
//  digitalWrite(LEDB, HIGH);
  data[count][0] = altitude;
  data[count][1] = accel;

  // 最高高度を更新
  if (altitude > highestAltitude) {
    highestAltitude = altitude;
    Serial.print("high");
    Serial.println(altitude);
  }
  Serial.println(altitude);

  

  //ここからメインコード 
  switch (state) {
    case CHECK_FALLING:
    data[count][2] = 0;
    Serial.println("falling");



//      digitalWrite(LEDB, LOW);
      // 現在高度が最高地点からfallDiatance以上落下したとき、かつ加速度がfallAccelThreshold以下になったときに，落下を検出
      if (abs(accel - fallAccelThreshold) > 2.0) {
        fallStartTime = millis();
        state = CHECK_LANDED;
        lastPressure = basePressure;
      }
      break;
      
    case CHECK_LANDED:
    data[count][2] = 1;

          Serial.println("landed");

//      digitalWrite(LEDR, LOW);
      // 100*10ms=1sec*landedIdleDuration秒以上の間，加速度がほぼ1.0G（自由落下終了=重力加速度のみ)，かつ加速度変化がほぼない，かつ気圧の変化がほぼない場合、着地と判断。
      // また，落下開始から一定時間経過した時は，無条件で着地と判断(強制分離)。
      if (abs(fallAccelThreshold - accel) <= 0.2 && abs(accel - lastAccel) <= landedAccelThreshold && abs(pressure - lastPressure) <= landedPressureThreshold){
        i++;
        j=0;
      }else if ((millis() - fallStartTime)/1000 >= ForcequitDuration) {
        i = landedIdleDuration*10;
        Serial.println(i);

      }else{
        j++;
        if (j==2) i=0,j=0;
      };
      delay(100);
      if (i <= 1) lastPressure = pressure; // 初回時の気圧を記憶
      if (i >= landedIdleDuration*10) state = EXIT;
      Serial.println("landed");
      break;

    case EXIT:
    data[count][2] = 2;
//    Serial.println("wrote in SD");
//    myFile = SD.open("test.txt", FILE_WRITE);
//    for(int i = 0; i < count; i++){
//
//      sd_write(data[i][0], data[i][1], data[i][2]);
//      delay(20);
//    }
//    myFile.close();

          Serial.println("done");
          delay(1000);
          return;
          

//      digitalWrite(LEDG, LOW);
    // while(1){

    // }
      
      
    //   // サーボモーターを180度回転
    //   state = COMPLETE;
    //   Serial.println("complete");
    //   ;

    case COMPLETE:
      break;

    
  }

  // 前回の加速度を更新
  lastAccel = accel;

  delay(1000);
  }
}
