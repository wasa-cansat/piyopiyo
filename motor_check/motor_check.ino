#include "motor.h"
#include <Servo.h>

Servo myservo; 

Motor motor;

void moveTowardsDestination();

void setup() {
  //Serial.begin(9600);
  //Serial.println("モーターの動作確認")
  delay(5000);
  myservo.attach(6);  // attaches the servo on pin 9 to the servo object

  myservo.write(40);
  delay(2000);
  myservo.write(180);
  delay(1000);
}

void loop() {
  while(1){
  moveTowardsDestination(); 
  }// 目的地に向かって移動
}


void moveTowardsDestination() {
  motor.forward(5000);  //それぞれ5秒動作確認
  motor.go_right(500);
  motor.go_left(500);
  motor.freeze(500);
}
