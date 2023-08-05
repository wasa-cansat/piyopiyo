#include <SD.h>

File myFile;

unsigned long time;  // 時間を記録


void sd_setup(){
  pinMode(10, OUTPUT);
  while(!SD.begin(10)){
    delay(1000);
  }
}

void sd_write(){
  myFile = SD.open("test.txt", FILE_WRITE);
  if (myFile){
    myFile.print("time: ");
    time = millis();
    myFile.print(String(time/1000)+" ");
    myFile.println("問題なし");
  }
  myFile.close();
}

void setup() {
  Serial.begin(115200);
  sd_setup();
}

void loop() {
  sd_write();
  delay(1000);
}
