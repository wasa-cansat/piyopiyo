#ifndef MAIN_H
#define MAIN_H

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include "HUSKYLENS.h"
#include "motor.h"
#include <SD.h>

File myFile;
Motor motor;
HUSKYLENS huskylens;
TinyGPSPlus GPS;

struct GPSData {
  double Latitude;
  double Longitude;
  double bearing;
  double distance;
};

//変更可
double destLatitude = 35.9064485;    // 目的地の緯度
double destLongitude = 139.6238548;   // 目的地の経度

//変更不可
SoftwareSerial gpsSerial(0, 1);  // RX_PIN, TX_PIN
unsigned long time;
bool firstTime = true;

void sd_setup();
void sd_write();
void sd
GPSData getGPSData();
void moveTowardsDestination();
void findObject();

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  sd_setup();
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
  moveTowardsDestination();
}


void sd_setup(){
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    delay(1000);
  }
}

void sd_write(){
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


GPSData getGPSData() {
  static LocationData data = { 0.0, 0.0, 0.0, 0.0 };
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
      if(data.latitude != 0.0){
        return data;
      }
    }
  }
}

void moveTowardsDestination() {
  previousData = getGPSData();
  motor.forward(20000);  //5秒間前進
  Serial.println("go straight");
  motor.freeze(100);
  nowData = getGPSData();
  double rot = atan((nowData.Longitude - previousData.Longitude)/(nowData.Latitude - previousData.Latitude))*180/PI;
  double destrot = atan((destLongitude - previousData.Longitude)/(destLatitude - previousData.Latitude))*180/PI;
  if (destrot>rot){
//    motor.go_left((int)(destrot-rot)*10);
    motor.go_left(1000);
    Serial.println("go left");
    Serial.println(rot);
    Serial.println(destrot);
    Serial.println(nowData.Longitude,15);
    Serial.println(nowData.Latitude, 15);
    Serial.println(previousData.Longitude, 15);
    Serial.println(nowData.Latitude, 15);
  }
  else{
//    motor.go_right((int)(rot-destrot)*10);
    motor.go_right(1000);
    Serial.println("go right");
    Serial.println(rot);
    Serial.println(destrot);
    Serial.println(nowData.Longitude, 15);
    Serial.println(nowData.Latitude, 15);
    Serial.println(previousData.Longitude, 15);
    Serial.println(nowData.Latitude, 15);
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
