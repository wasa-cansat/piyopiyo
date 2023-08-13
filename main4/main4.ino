//前回実験の際に使用していたコード

#ifndef MAIN_H

#define MAIN_H
//重複定義の回避
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "HUSKYLENS.h"
#include "motor.h"
//#include <Arduino_LSM9DS1.h>
#include <SD.h>
File myFile;
Motor motor;
HUSKYLENS huskylens;
TinyGPSPlus gps;
SoftwareSerial gpsSerial(0, 1);  // RX_PIN, TX_PIN
double destLatitude = 35.9064485;    // 目的地の緯度
double destLongitude = 139.6238548;   // 目的地の経度
struct LocationData {
  double currentLatitude;
  double currentLongitude;
  double bearing;
  double distance;
};
LocationData previousData;    // 前のデータを保持する変数
LocationData nowData;       // 現在のデータを保持する変数
float previousTheta;
float nowTheta;
float adjustedTheta;
bool firstTime = true; // 初回のみ
unsigned long time;  // for sdcard
//落下検出　坂
void sd_setup();
void sd_GPSwrite(double longitude, double latitude);
void sd_ROTATEwrite(float rotate);
LocationData getGPSData();
// void calibrate();//地磁気のためのキャリブレーション
void moveTowardsDestination();
void findObject();
void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  Wire.begin();
//  while (!huskylens.begin(Wire))
//  {
//        Serial.println(F("Begin failed!"));
//        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>I2C)"));
//        Serial.println(F("2.Please recheck the connection."));
//        delay(100);
//  }
}
void loop() {
        Serial.println("GPS");
  //落下検知
  if(firstTime){
    motor.forward(3000); //3秒間前進 とりあえず動いてみる
    motor.freeze(100);
    firstTime = false;
  }
  if (huskylens.request()) {
      if (huskylens.isLearned()) {
          if (huskylens.available()) {
            findObject();
            return;
          }
      }
  }
    do{
                Serial.println("now in gps");
     nowData = getGPSData();
    }while(nowData.currentLatitude == 0.0);
          moveTowardsDestination();
}
//落下検出　坂
void sd_setup(){
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    delay(1000);
  }
}
void sd_GPSwrite(double longitude, double latitude){
  myFile = SD.open("log.txt", FILE_WRITE);
  if (myFile){
    myFile.print("time: ");
    time = millis();
    myFile.print(String(time/1000));
    myFile.print(" longitude: ");
    myFile.print(String(longitude));
    myFile.print(" longitude: ");
    myFile.println(String(latitude));
  }
  myFile.close();
}
void sd_ROTATEwrite(float rotate){
  myFile = SD.open("log.txt", FILE_WRITE);
  if (myFile){
    myFile.print("time: ");
    time = millis();
    myFile.print(String(time/1000));
    myFile.print(" azimuth: ");
    myFile.println(String(rotate));
  }
  myFile.close();
}
LocationData getGPSData() {
  static LocationData data = {0.0, 0.0, 0.0, 0.0};
  while (gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    gps.encode(c);
    if(!gps.location.isUpdated()) continue;
    data.currentLatitude = gps.location.lat();
    data.currentLongitude = gps.location.lng();
    data.bearing = TinyGPSPlus::courseTo(                //from current to dest (North=0, West=270)
        data.currentLatitude, data.currentLongitude,
        destLatitude, destLongitude
    );
    data.distance = TinyGPSPlus::distanceBetween(
        data.currentLatitude, data.currentLongitude,
        destLatitude, destLongitude
    );
    return data;
  }
}
void moveTowardsDestination() {
  previousData = getGPSData();
  motor.forward(20000);  //5秒間前進
  Serial.println("go straight");
  motor.freeze(100);
  nowData = getGPSData();
  double rot = atan((nowData.currentLongitude - previousData.currentLongitude)/(nowData.currentLatitude - previousData.currentLatitude))*180/PI;
  double destrot = atan((destLongitude - previousData.currentLongitude)/(destLatitude - previousData.currentLatitude))*180/PI;
  if (destrot>rot){
//    motor.go_left((int)(destrot-rot)*10);
    motor.go_left(1000);
    Serial.println("go left");
    Serial.println(rot);
    Serial.println(destrot);
    Serial.println(nowData.currentLongitude,15);
    Serial.println(nowData.currentLatitude, 15);
    Serial.println(previousData.currentLongitude, 15);
    Serial.println(nowData.currentLatitude, 15);
  }
  else{
//    motor.go_right((int)(rot-destrot)*10);
    motor.go_right(1000);
    Serial.println("go right");
        Serial.println(rot);
    Serial.println(destrot);
    Serial.println(nowData.currentLongitude, 15);
    Serial.println(nowData.currentLatitude, 15);
    Serial.println(previousData.currentLongitude, 15);
    Serial.println(nowData.currentLatitude, 15);
  }
}
void findObject(){
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
                  Serial.println("stop");
                  while (1) {
                    motor.freeze(2000);
                  }
              }
              if (left < 70) {
                  motor.go_left(100);
                  motor.freeze(0);
              } else if (right > 250) {
                  motor.go_right(100);
                  motor.freeze(0);
              } else {
                  motor.forward(1000);
                  motor.freeze(0);
              }
              delay(100);
}
#endif
