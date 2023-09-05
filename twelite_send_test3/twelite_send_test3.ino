void setup() {
Serial2.begin(38400);
Serial.begin(38400);
pinMode(LED, OUTPUT);
}
void loop() {
byte recv [60] = {};
byte count     = 0;
//receive data
while (TWE.available())
{
digitalWrite(LED, HIGH);
recv [count] = TWE.read();
count++;
}
if (count > 0)
{
for (byte i = 0 ; i < count ; i++) Serial.print((char)(recv[i]));
}
digitalWrite(LED, LOW);
}
