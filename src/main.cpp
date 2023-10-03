#include <Arduino.h>
#include <M5CoreS3.h>

void setup()
{
  M5.begin(115200);
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.println("hello world");
  M5.Lcd.println("hello world");
  delay(10000);
}
