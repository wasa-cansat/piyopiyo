#include "HUSKYLENS.h"
#include "motor.h"

Motor motor;
HUSKYLENS huskylens;

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
