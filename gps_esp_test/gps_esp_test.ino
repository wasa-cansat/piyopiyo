#include<TinyGPS++.h>

TinyGPSPlus gps;
int RX_PIN = 22;
int TX_PIN = 23;
//HardwareSerial Serial2(2);

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);//シリアルモニタに表示
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);//gpsモジュールからの受信
}

void loop() {
  while (Serial1.available()>0){
    char c = Serial1.read();
    gps.encode(c);
    if(gps.location.isUpdated()){
      Serial.print("LAT: "); Serial.println(gps.location.lat(),16);//シリアルモニタに表示するためのplint
      Serial.print("LONG: "); Serial.println(gps.location.lng(),16);
    }
  }
}
