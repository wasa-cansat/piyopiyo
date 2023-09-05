#include <TinyGPS++.h>

TinyGPSPlus gps;

float gps_lat; //緯度
float gps_longt; //経度 

int RX_PIN = 23;
int TX_PIN = 22;



struct LocationData {
  double latitude;
  double longitude;
};


byte start_byte[5] = {':', '7', '8', '0', '1'};
byte end_byte[3] = {'X', '\r', '\n'};

LocationData getGPSData();

void setup()
{
  // シリアルポート開始
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial2.begin(38400);
}

void loop() {
  //GPSデータ取得
  LocationData nowData = getGPSData();
  
  //緯度経度をdoubleからcharの配列に変換
  char latitude_char[14]; 
  char longitude_char[14];
  dtostrf(nowData.latitude, 13, 9, latitude_char);
  dtostrf(nowData.longitude, 13, 9, longitude_char);

  //ヘッダ、コマンドなど送信
  for(int i = 0; i < 5; i++){
    Serial2.write(start_byte[i]);
  }
  //緯度送信
  for(int i = 0; i < 14; i++){
    Serial2.write(latitude_char[i]);
  }
  //軽度送信
  for(int i = 0; i < 14; i++){
    Serial2.write(longitude_char[i]);
  }
  //チェックサム、終端など送信
  for(int i = 0; i < 3; i++){
    Serial2.write(end_byte[i]);
  }
  
  
 
}

LocationData getGPSData() {
  static LocationData nowData = {0.0, 0.0};
   if (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
    if (gps.location.isUpdated()) {

      nowData.latitude = gps.location.lat();
      nowData.longitude = gps.location.lng();
      Serial.print("LAT:  "); Serial.println(nowData.latitude,9);
      Serial.print("LONG: "); Serial.println(nowData.longitude,9);
    }
  }
  return nowData;
}
