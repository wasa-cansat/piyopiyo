#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include "motor.h"
#include <Adafruit_BMP280.h>
#include <Servo.h>

Servo myservo; 
Motor motor;

void setup() {
  // put your setup code here, to run once:
  myservo.attach(6);
  myservo.write(180);
  delay(100);
  myservo.write(90);
  delay(100);
  myservo.write(160);

    //モーター用
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000*10);
  myservo.write(40);
  delay(2000);
  myservo.write(180);
  delay(1000);
  while(1){
    motor.forward(30);
  }
}
