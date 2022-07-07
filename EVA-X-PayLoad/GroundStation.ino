#include "Arduino.h"
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
SoftwareSerial loraSerial(10, 11);
LoRa_E32 e32ttl(&loraSerial);

void setup()
{
  Serial.begin(9600);
  delay(100);
  e32ttl.begin();
  Serial.println();
}
typedef struct {
byte altitude[5];
byte latitude[10];
byte longtitude[10];
} Signal;
Signal rocket;

void loop()
{
  if (e32ttl.available()  > 1){
    ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Signal));
    rocket = *(Signal*) rsc.data;
    Serial.print(F("Altitude: "));
    Serial.print(*(float*)rocket.altitude,6);

    Serial.print(F(" Latitude: "));
    Serial.println(*(float*)rocket.latitude,6);

    Serial.print(F(" Longitude"));
    Serial.println(*(float*)rocket.longtitude,6);
    rsc.close();
  }
}
