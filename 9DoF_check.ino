#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup(void)
{
  Serial.begin(9600);
  Serial2.begin(115200);
  if (!bno.begin())
  {
    Serial.print("no conection");
    while (1);
  }

  delay(1000);
  bno.setExtCrystalUse(true);
  displaySensorDetails();
}

void loop(void)
{
  sensors_event_t event;
  bno.getEvent(&event);
  float ox = 360 - (float)event.orientation.x;
  float oy = (float)event.orientation.y;
  float oz = (float)event.orientation.z;
  Serial.printf("%4.3f,%4.3f,%4.3f,", ox, oy, oz);

  imu::Quaternion quat = bno.getQuat();
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  Serial.print(sys, DEC);
  Serial.print(F(", "));
  Serial.print(gyro, DEC);
  Serial.print(F(", "));
  Serial.print(accel, DEC);
  Serial.print(F(", "));
  Serial.print(mag, DEC);
  Serial.println(F(""));

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
