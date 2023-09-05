void setup() {
Serial2.begin(38400);
}
void loop() {
#define DATA_SIZE 10
byte DATA[DATA_SIZE] = {':', '7', '8', '0', '1', '3', '5', 'X', '\r', '\n'};
for (byte i = 0 ; i < DATA_SIZE ; i++) Serial2.write(DATA[i]);
delay(300);
}
