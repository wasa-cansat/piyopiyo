
/* README (2023.08.08 Koya Saka)
- パラメーター
  fallDistance,landedIdleDuration,ForcequitDurationを調整して下さい。
  ※Threshold関連は調整済みなので，あまりいじらない方がいいかも。
  

- ステータスLEDの意味 
  電源が繋がると，ボード上のLEDが青色に光る。（赤色に光ったら，ボード上のIMUか気圧センサの初期化エラー。）
  その後，
  落下検知中＝青色
  着地検知中（待機）＝赤色
  分離＝緑色
  と光るので，LEDを見て今の処理状態を確認して下さい。
*/
#include <SD.h>
const int chipSelect = 10;

double data[30][3];
int count = -1;


File myFile;

#include <Arduino_LPS22HB.h>
#include <Arduino_LSM9DS1.h>
#include <Servo.h>

#include <Wire.h>
//#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



void sd_setup(){
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    Serial.println("no sd");
    delay(1000);
  }
}

void sd_write(double height, double accel, double stat){
  if (myFile){

      myFile.print("height");
      myFile.println(height);
      myFile.print("accel");
      myFile.println(accel);
      myFile.print("status");
      myFile.println(stat);
    }
  else{
    Serial.println("cannot write to file");
  }
}

// 落下距離(m)の閾値[30.0 ±0.6]
const float fallDistance = 1.0;
// 自由落下時の加速度(m/s2)の閾値[0.5]
const float fallAccelThreshold = 9.68;
// 着地後静止時の加速度変化(m/s2)の誤差範囲(理論値は0)[0.1]
const float landedAccelThreshold = 0.1;
// 着地後静止時の気圧変化(Pa)の誤差範囲(理論値は0）[30]
const float landedPressureThreshold = 30;
// 着地後静止時の大まかな検出待機時間(自然数sec)[15]
const unsigned long landedIdleDuration = 15;
// 落下開始から強制分離までの時間(sec)[90]
const unsigned long ForcequitDuration = 90;


//BME280
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
Adafruit_BMP280 bmp; // I2C

//MNO055
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

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
// サーボモーター用のオブジェクトを生成


void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
    sd_setup();

  
  Serial.begin(9600);
  Serial.println("start serial");
//  while (!Serial);
  // set the LEDs pins as outputs
//  pinMode(LEDR, OUTPUT);
//  pinMode(LEDG, OUTPUT);
//  pinMode(LEDB, OUTPUT);

  //BME280
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

  //BNO055
  if(!bno.begin())
  {

    Serial.println("no bno");
//    digitalWrite(LEDR, LOW);
    while(1);
  }



  // 地表の気圧を読み取る
  basePressure = bmp.readPressure();
  lastPressure = basePressure;
}


void loop() {

  Serial.println(count);
  if(count >= 30){
    count = 0;
    Serial.println("wrote in SD");
    delay(1000);

    myFile = SD.open("test.txt", FILE_WRITE);
    for(int i = 0; i < 30; i++){

      sd_write(data[i][0], data[i][1], data[i][2]);
      delay(20);
    }
    myFile.close();

  }
  else{
    count++;
  }

  
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
    data[count][3] = 0;



//      digitalWrite(LEDB, LOW);
      // 現在高度が最高地点からfallDiatance以上落下したとき、かつ加速度がfallAccelThreshold以下になったときに，落下を検出
      if ((highestAltitude - altitude >= fallDistance) && (accel >= fallAccelThreshold)) {
        fallStartTime = millis();
        state = CHECK_LANDED;
        lastPressure = basePressure;
      }
      break;
      
    case CHECK_LANDED:
    data[count][3] = 1;

          Serial.println("landed");

//      digitalWrite(LEDR, LOW);
      // 100*10ms=1sec*landedIdleDuration秒以上の間，加速度がほぼ1.0G（自由落下終了=重力加速度のみ)，かつ加速度変化がほぼない，かつ気圧の変化がほぼない場合、着地と判断。
      // また，落下開始から一定時間経過した時は，無条件で着地と判断(強制分離)。
      if (abs(fallAccelThreshold - accel) <= 0.2 && abs(accel - lastAccel) <= landedAccelThreshold && abs(pressure - lastPressure) <= landedPressureThreshold){
        i++;
        j=0;
      }else if ((millis() - fallStartTime)/1000 >= ForcequitDuration) {
        i = landedIdleDuration*10;
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
    data[count][3] = 2;

          Serial.println("done");

//      digitalWrite(LEDG, LOW);
    while(1){

    }
      
      
      // サーボモーターを180度回転
      state = COMPLETE;
      Serial.println("complete");
      break;

    case COMPLETE:
      break;

    
  }

  // 前回の加速度を更新
  lastAccel = accel;

  delay(700);
}
