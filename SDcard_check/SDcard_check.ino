#include <SD.h>

File myFile;

unsigned long time;  // 時間を記録

int n = 0;

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
  else{
    Serial.println("cannot write to file");
  }
  myFile.close();
}

void setup() {
  Serial.begin(115200);
  Serail.println("start")
  sd_setup();
  Serial.println("done");
}

void loop() {
  sd_write();
  Serial.println(n); n++;
  delay(1000);
}
