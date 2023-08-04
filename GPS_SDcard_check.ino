#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SD.h>

File myFile;
TinyGPSPlus gps;
double destLatitude = 35.9064485;    // 目的地の緯度
double destLongitude = 139.6238548;   // 目的地の経度

int RX_PIN = 0;
int TX_PIN = 1;

SoftwareSerial gpsSerial(RX_PIN, TX_PIN);

struct LocationData {
  double currentLatitude;
  double currentLongitude;
  double bearing;
  double distance;
};

void sd_setup(){
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    delay(1000);
  }
}

void sd_GPSwrite(double longitude, double latitude){
  myFile = SD.open("log.txt", FILE_WRITE);
  if (myFile){
    myFile.print("time: ");
    time = millis();
    myFile.print(String(time/1000));
    myFile.print(" longitude: ");
    myFile.print(String(longitude,9));
    myFile.print(" longitude: ");
    myFile.println(String(latitude,9));
  }
  myFile.close();
}

LocationData getGPSData() {
  static LocationData data = {0.0, 0.0, 0.0, 0.0}; 

  while (gpsSerial.available() > 0) {
    
    char c = gpsSerial.read();
    gps.encode(c);
    
    if(!gps.location.isUpdated()) continue;
    
    data.currentLatitude = gps.location.lat();
    data.currentLongitude = gps.location.lng();

    data.bearing = TinyGPSPlus::courseTo(
        data.currentLatitude, data.currentLongitude,
        destLatitude, destLongitude
    );

    data.distance = TinyGPSPlus::distanceBetween(
        data.currentLatitude, data.currentLongitude,
        destLatitude, destLongitude
    );

    return data; // GPSデータが更新されたら値を返す
  }
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  sd_setup();
  Serial.println("【start】");
}

void loop() {
  unsigned long startTime = millis();
  LocationData result = getGPSData();
  Serial.print(String(startTime / 1000)+"秒　");


  //if(result.currentLatitude == 0.0){  
    //Serial.println("Please wait for a moment");
    //delay(1000);
  //}

  //else{

  Serial.print("現在地（");
  Serial.print(result.currentLatitude,2);
  Serial.print(",");
  Serial.print(result.currentLongitude,2);
  Serial.print(") ");
  
  Serial.print("方位: ");
  Serial.print(result.bearing, 2);
  Serial.print("°  ");
  Serial.print("距離: ");
  Serial.print(result.distance, 2);
  Serial.println("m");

  sd_GPSwrite( result.longitude, result.latitude);
  
  delay(1000); // Check GPS data every 1 second
  //}
}
