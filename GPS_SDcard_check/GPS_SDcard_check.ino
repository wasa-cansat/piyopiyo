
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
  double latitude;
  double longitude;
  double bearing;
  double distance;
};

void sd_setup(){
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    delay(1000);
  }
}

void sd_GPSwrite(double latitude, double longitude, double bearing, double distance){
  myFile = SD.open("log.txt", FILE_WRITE);
  if (myFile){
    myFile.print("time: ");
    unsigned long time = millis();
    myFile.println(String(time/1000));
    if(latitude != 0.0){
      myFile.print(" Latitude: ");
      myFile.print(String(latitude,9));
      Serial.println(latitude, 9);
      myFile.print(" longitude: ");
      myFile.print(String(longitude,9));
      Serial.println(longitude, 9);
      myFile.print(" bearing: ");
      myFile.print(String(bearing,9));
      Serial.println(bearing, 9);
      myFile.print(" distance: ");
      myFile.println(String(distance,9));
      Serial.println(distance, 9);
    }
  }
  myFile.close();
}

void getGPSData() {
  static LocationData data = {0.0, 0.0, 0.0, 0.0}; 

  while (gpsSerial.available() > 0) {
          Serial.println("in GPS");

    
    char c = gpsSerial.read();
    gps.encode(c);
    
    if (gps.location.isUpdated()) {

    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();

    data.bearing = TinyGPSPlus::courseTo(
        data.latitude, data.longitude,
        destLatitude, destLongitude
    );

    data.distance = TinyGPSPlus::distanceBetween(
        data.latitude, data.longitude,
        destLatitude, destLongitude
    );

    sd_GPSwrite(data.longitude, data.latitude, data.bearing, data.distance);
  }
}
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);
  sd_setup();
}

void loop() {
  getGPSData();

  //delay(1000); // Check GPS data every 1 second
  //}
}
