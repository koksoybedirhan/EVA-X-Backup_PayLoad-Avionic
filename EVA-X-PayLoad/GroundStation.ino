#include "Arduino.h"
#include "LoRa_E32.h"
#include <SoftwareSerial.h>
SoftwareSerial loraSerial(10, 11);
LoRa_E32 e32ttl(&loraSerial);
String value = ",";

void setup()
{
  Serial.begin(9600);
  delay(100);
  e32ttl.begin();
  Serial.println();

}
typedef struct {
byte alt[7];
byte lat[10];
byte lng[10];
byte irt[8];
byte sic[6];
byte ne[6];
byte bas[8];
} Signal;
Signal data;

void loop()
{
  if (e32ttl.available()  > 1){
    ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(Signal));
    data = *(Signal*) rsc.data;
    Serial.print(*(float*)data.alt,1);
    Serial.print(value);
    Serial.print(*(float*)data.lat,6);
    Serial.print(value);
    Serial.print(*(float*)data.lng,6);
    Serial.print(value);
    Serial.print(*(float*)data.irt,2);
    Serial.print(value);
    Serial.print(*(float*)data.sic,2);
    Serial.print(value);
    Serial.print(*(float*)data.ne,2); 
    Serial.print(value);
    Serial.print(*(float*)data.bas,2);  
    Serial.println("/");
    rsc.close();
  }
}
