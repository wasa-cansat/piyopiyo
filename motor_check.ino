#include "motor.h"

Motor motor;

void moveTowardsDestination();

void setup() {
  //Serial.begin(9600);
  //Serial.println("モーターの動作確認")
}

void loop() {
  moveTowardsDestination();  // 目的地に向かって移動
}


void moveTowardsDestination() {
  motor.forward(5000);  //それぞれ5秒動作確認
  motor.backward(5000);
  motor.go_right(5000);
  motor.go_left(5000);
  motor.freeze(5000);
}