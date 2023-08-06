#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

//GPS関連
TinyGPSPlus gps;
double destLatitude = 35.722309;    // 目的地の緯度
double destLongitude = 139.686798;  // 目的地の経度

int RX_PIN = 1;
int TX_PIN = 0;

SoftwareSerial gpsSerial(RX_PIN, TX_PIN);

struct LocationData {
  double latitude;
  double longitude;
  double bearing;
  double distance;
};

void startTime();
double getGPSData();



//bno055関連
double maxx;
double maxy;
/* Set the delay between fresh samples */
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);



//モーター関連
void right();
void left();
void motor_stop();



void setup(void)
{

  //モーター用
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);

  //GPS
  Serial.begin(115200);
  gpsSerial.begin(9600);
  pinMode(RX_PIN, INPUT);
  pinMode(TX_PIN, OUTPUT);

  
  //bno055
  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

}

void loop(void)
{


  double error = 0;
  
  while(1){

    double bearing = getGPSData();
    Serial.println(bearing);

    sensors_event_t orientationData, magnetometerData;

    bno.getEvent(&orientationData);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
    double x = orientationData.orientation.x;
    double y = orientationData.orientation.y;
    double z = orientationData.orientation.z;
    double mag_x = magnetometerData.magnetic.x;
    double mag_y = magnetometerData.magnetic.y;
    double mag_z = magnetometerData.magnetic.z;
    if(abs(mag_y + 0.06) < 0.0001 || abs(mag_y + 8.06) < 0.0001 || (mag_x + mag_y + mag_z) > 2000){
//        Serial.println("not valid");
    }
    else{
          error = atan2(mag_y, mag_x)*180/PI + 180 - x;
//        Serial.print("mag_x = ");
//        Serial.println(atan2(mag_y, mag_x)*180/PI + 180);
//        Serial.print("error: ");
//        Serial.println(error);
    }
    double cal_x = (x + error)<360 ? x + error: x + error - 360.0;
    Serial.print("x = ");
    Serial.println(cal_x);

//    Serial.println(data.bearing);
    if(abs(cal_x - bearing) < 10.0)
    {
      motor_stop();
      Serial.println("stop");
    }
    else if(((bearing - cal_x) > 0 ? bearing - cal_x: bearing + 360 - cal_x) < 180 )
    {
      Serial.println("right");
      right();
    }
    else
    {
      left();
      Serial.println("left");
    }
  }
}

void left()
{
  digitalWrite(2, HIGH);
  digitalWrite(3, LOW);
}

void right()
{
  digitalWrite(2, LOW);
  digitalWrite(3, HIGH);
}

void motor_stop()
{
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);
}

//GPS関連
void startTime() {
  unsigned long startTime = millis();
  Serial.print(String(startTime / 1000) + "秒　　　");
}

double getGPSData() {
  static LocationData data = { 0.0, 0.0, 0.0, 0.0 };
    Serial.println("in GPS");

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
      
//      unsigned long startTime = millis();
//      Serial.println(String(startTime / 1000) + "秒　　　");
      if(data.latitude != 0.0){
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
        return data.bearing;
      }
    }
  }
}
