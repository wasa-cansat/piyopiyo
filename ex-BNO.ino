#include <Wire.h>

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  while(!Serial);
}

bool slavePresent(byte adr)
{
  Wire.beginTransmission(adr);
  return(Wire.endTransmission() == 0);
}

void loop()
{
  Serial.println("I2C slave dcvice list.");
  for(byte adr = 1; adr<127; adr++){
    if (slavePresent(adr)){
      if (adr<16) Serial.print("0");
      Serial.print(adr, HEX);
      Serial.print(" ");
    }
  }
  Serial.println("\nDone.");
  delay(5000);
}
