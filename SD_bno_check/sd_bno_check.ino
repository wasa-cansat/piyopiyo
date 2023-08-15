#include <SD.h>
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
double maxx;
double maxy;

File myFile;


uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);




void sd_setup(){
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    delay(1000);
  }
}

void sd_write(double x){
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile){
    myFile.print("angle");
    myFile.println(x);
    Serial.print("angle");
    Serial.println(x);
    myFile.println("問題なし");
    Serial.println("問題なし");
  }
  else{
    Serial.println("cannot write to file");
  }
  myFile.close();
}

void setup() {
  
  Serial.begin(9600);
  Serial.println("start");
  sd_setup();
  Serial.println("done");

  while (!Serial) delay(10);  // wait for serial port to open!

  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  delay(1000);
  
}
double error;

void loop() {
  sensors_event_t orientationData, magnetometerData;

    bno.getEvent(&orientationData);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
  //
  //printEvent(&orientationData);
  //  printEvent(&angVelocityData);
  //  printEvent(&linearAccelData);
  
    double x = orientationData.orientation.x;
    double y = orientationData.orientation.y;
    double z = orientationData.orientation.z;
    double mag_x = magnetometerData.magnetic.x;
    double mag_y = magnetometerData.magnetic.y;
    double mag_z = magnetometerData.magnetic.z;
    if(abs(mag_y + 0.06) < 0.0001 || abs(mag_y + 8.06) < 0.0001 || (mag_x + mag_y + mag_z) > 2000){
        Serial.println("not valid");
    }
    else{
        Serial.print("mag_x = ");
        Serial.println(atan2(mag_y, mag_x)*180/PI + 180);
        error = atan2(mag_y, mag_x)*180/PI + 180 - x;
        Serial.print("error: ");
        Serial.println(error);
    }
    double cal_x = (x + error)<360 ? x + error: x + error - 360.0;
    Serial.print("x = ");
    Serial.println(cal_x);

  sd_write(cal_x);
  delay(1000);
  
}
