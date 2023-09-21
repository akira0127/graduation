#include <Arduino.h>
#include <M5CoreS3.h>

// put function declarations here:
int myFunction(int, int);

void setup()
{
  // put your setup code here, to run once:
  int result = myFunction(2, 3);

  Serial.begin(115200);
}

void loop()
{
  // put your main code here, to run repeatedly:
  Serial.println("hello world");
  M5.Lcd.println("hello world");
  delay(10000);
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}