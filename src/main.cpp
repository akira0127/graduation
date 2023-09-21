#include <Arduino.h>
#include <M5CoreS3.h>
#include <SoftwareSerial.h>

// put function declarations here:
int myFunction(int, int);

void setup()
{
  // put your setup code here, to run once:
  int result = myFunction(2, 3);
  serial.begin(115200);
}

void loop()

{
  // put your main code here, to run repeatedly:
  serial.println("hello");
  delay(5000);
}

// put function definitions here:
int myFunction(int x, int y)
{
  return x + y;
}