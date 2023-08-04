#include <TinyGPS++.h>
#include <SoftwareSerial.h>

TinyGPSPlus gps;
double destLatitude = 35.9064485;    // 目的地の緯度
double destLongitude = 139.6238548;  // 目的地の経度

int RX_PIN = 0;
int TX_PIN = 1;

SoftwareSerial gpsSerial(RX_PIN, TX_PIN);

struct LocationData {
  double latitude;
  double longitude;
  double bearing;
  double distance;
};

void startTime() {
  unsigned long startTime = millis();
  Serial.print(String(startTime / 1000) + "秒　　　");
}

void getGPSData() {
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
      
      unsigned long startTime = millis();
      Serial.println(String(startTime / 1000) + "秒　　　");
      if(data.Latitude != 0.0){
        Serial.print("現在地（");
        Serial.print(data.latitude, 6);
        Serial.print(",");
        Serial.print(data.longitude, 6);
        Serial.print(") ");
    
        Serial.print("方位: ");
        Serial.print(data.bearing, 6);
        Serial.print("°  ");
        Serial.print("距離: ");
        Serial.print(data.distance, 6);
        Serial.println("m");
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  Serial.println("【start】");
}

void loop() {

  getGPSData();

  //delay(1000);  // Check GPS data every 1 second
  //}
}
