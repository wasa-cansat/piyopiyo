#ifndef MOTOR_H
#define MOTOR_H

// モータのピン番号
const int IN1 = 5;
const int IN2 = 4;
const int IN3 = 3;
const int IN4 = 2;

class Motor {
public:
  Motor();
  void forward(int t);
  void backward(int t);
  void go_left(int t);
  void go_right(int t);
  void freeze(int t);
};

#endif