#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>
#include "motor.h"
#include "HUSKYLENS.h"

#include <Servo.h>


//servo
Servo myservo; 



//HUSKYLENS
HUSKYLENS huskylens;
void findObject();
void findObjectSetup();
bool cam_sw = false;

//GPS関連
TinyGPSPlus gps;
double destLatitude = 35.705287;    // 目的地の緯度
double destLongitude = 139.714019;  // 目的地の経度

int RX_PIN = 0;
int TX_PIN = 1;

SoftwareSerial gpsSerial(RX_PIN, TX_PIN);

struct LocationData {
  double latitude;
  double longitude;
  double bearing;
  double distance;
};

void startTime();
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
void sd_GPSwrite(double latitude, double longitude, double bearing, double distance, double angle);


unsigned long time, previous_time;


void setup(void)
{

  //SD用
  sd_setup();

  //servo
  myservo.attach(6);
  myservo.write(40);
  delay(2000);
  myservo.write(180);
  delay(1000);


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


  Serial.println("now in loop");
  double cal_x = 0;
  double error = 0;
  LocationData now_data = {0.0, 0.0, 0.0, 20.0};

  time = millis();

  
  while(1){

    if (huskylens.request()) {
      if (huskylens.isLearned()) {
          if (huskylens.available()) {
            findObject();
            continue;
          }
          else if(cam_sw){
            findObjectSetup();
          }
      }
    }

    now_data = getGPSData(cal_x);
    Serial.println(now_data.bearing);
    previous_time = millis();
    if(previous_time -time > 100){



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
      Serial.println("stop");
    }
    else if(((now_data.bearing - cal_x) > 0 ? now_data.bearing - cal_x: now_data.bearing + 360 - cal_x) < 180 )
    {
      motor.go_right(0);
      Serial.println("right");
    }
    else
    {
      motor.go_left(0);
      Serial.println("left");
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
        sd_GPSwrite(data.latitude, data.longitude, data.bearing, data.distance, angle);
        return data;
      }
    }
  }
}

//SD用

void sd_setup(){
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    delay(1000);
    Serial.println("sd setup failed");
  }
}

void sd_GPSwrite(double latitude, double longitude, double bearing, double distance, double angle){
  myFile = SD.open("LOG.txt", FILE_WRITE);
  if (myFile){
    myFile.print("time: ");
    unsigned long gps_time = millis();
    myFile.println(String(gps_time/1000));
    if(latitude != 0.0){
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
    }
  }
  else{
    Serial.println("cannot open file");
  }
  myFile.close();
}

// void sd_GPSwrite(){
//   myFile = SD.open("test.txt", FILE_WRITE);
//   Serial.println("try to access sd-card");
//   if (myFile){
//     myFile.print("time: ");
//     unsigned long gps_time = millis();
//     myFile.print(String(gps_time/1000)+" ");
//     myFile.println("問題なし");
//   }
//   else{
//     Serial.println("cannot write to file");
//   }
//   myFile.close();
// }

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
