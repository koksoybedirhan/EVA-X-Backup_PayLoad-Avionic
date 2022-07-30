#include <Arduino.h>
#include <SoftwareSerial.h>
SoftwareSerial dorji(11,10);

void setup() 
{
  Serial.begin(9600);
  dorji.begin(9600);
}

void loop() 
{
  if(dorji.available()>0) 
  {    
    Serial.write(dorji.read());
  }
  delay(10);
}
