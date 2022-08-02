//kutuphaneler
#include "Arduino.h"
#include "LoRa_E32.h"
#include "TinyGPS++.h"
#include <SoftwareSerial.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define denizseviyesi (1006.25) //aksaraya göre hesaplanacak.

unsigned long delayTime;

//kutuphane tanimlari
SoftwareSerial portLora(19, 18);
LoRa_E32 e32ttl(&portLora);
TinyGPSPlus gps;
SimpleKalmanFilter kalmanIrtifa(1, 1, 0.01);
Adafruit_BME280 bme; 

float sicaklik, basinc, basincirtifa, kalman, nem;

//lora paketi
typedef struct {
byte alt[7];
byte lat[10];
byte lng[10];
byte irt[8];
byte sic[6];
byte ne[6];
} Signal;
Signal data;

void setup() 
{
  Serial.begin(9600);
  Serial2.begin(9600);
  e32ttl.begin();
  bme.begin(); 
  delay(500); 
}

void loop() 
{
  while(Serial2.available())
  {
    if(gps.encode(Serial2.read()))
    {
      sicaklik = bme.readTemperature();
      basinc = bme.readPressure() / 100.0F;
      basincirtifa = bme.readAltitude(denizseviyesi);
      nem = bme.readHumidity();
      String msg = Serial2.readStringUntil('\r');
      //Serial.println(msg);
      Serial.print("Latitude:"); Serial.print(gps.location.lat(), 6); Serial.print(" ");
      Serial.print("Longitude:"); Serial.print(gps.location.lng(), 6); Serial.print(" ");
      Serial.print("Altitude:"); Serial.print(gps.altitude.meters(), 6); Serial.println(" ");
      Serial.print("Sicaklik: "); Serial.print(sicaklik); Serial.print(" derece ");
      Serial.print("Basinc: "); Serial.print(basinc); Serial.print(" hPa ");
      Serial.print("Basinc İrtifa: "); Serial.print(basincirtifa); Serial.print(" metre ");
      Serial.print("Nem Yuzdesi: "); Serial.print(nem); Serial.print("% ");
      *(float*)(data.alt) = (float)gps.altitude.meters();
      *(float*)(data.lat) = (float)gps.location.lng();
      *(float*)(data.lng) = (float)gps.location.lat();
      *(float*)(data.irt) = (float)basincirtifa;
      *(float*)(data.sic) = (float)sicaklik;
      *(float*)(data.ne) = (float)nem;
      ResponseStatus rs = e32ttl.sendFixedMessage(0, 11, 21, &data, sizeof(Signal));
      Serial.println(rs.getResponseDescription());
      delay(4000);
    }
  }
}
