//kutuphaneler
#include "Arduino.h"
#include "LoRa_E32.h"
#include "TinyGPS++.h"
#include <SoftwareSerial.h>
#include <Adafruit_BMP280.h>
#include <SimpleKalmanFilter.h>

//kutuphane tanimlari
SoftwareSerial portLora(19, 18);
LoRa_E32 e32ttl(&portLora);
TinyGPSPlus gps;
SimpleKalmanFilter kalmanIrtifa(1, 1, 0.01);
Adafruit_BMP280 bmp;

float sicaklik, basinc, basincirtifa, kalman, s, i; 
float denizbasinc = 1001.25; //aksaraya göre hesaplanacak.
float alti;

//lora paketi
typedef struct {
byte alt[7];
byte lat[10];
byte lng[10];
byte irt[8];
byte sic[6];
} Signal;
Signal data;

void setup() 
{
  Serial.begin(9600);
  delay(500);
  Serial2.begin(9600);
  delay(500);
  e32ttl.begin();
  bmp.begin(0x77,0x60);
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,    
                  Adafruit_BMP280::SAMPLING_X2,    
                  Adafruit_BMP280::SAMPLING_X16,  
                  Adafruit_BMP280::FILTER_X16,      
                  Adafruit_BMP280::STANDBY_MS_500); 
}

void loop() 
{
  s = sicak();
  i = irtifa();
  while(Serial2.available())
  {
    if(gps.encode(Serial2.read()))
    {
      String msg = Serial2.readStringUntil('\r');
      //Serial.println(msg);
      Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
      Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
      Serial.print("ALT="); Serial.println(gps.altitude.meters(), 6);
      Serial.println(s); Serial.println(i);
      *(float*)(data.alt) = (float)gps.altitude.meters();
      *(float*)(data.lat) = (float)gps.location.lng();
      *(float*)(data.lng) = (float)gps.location.lat();
      *(float*)(data.irt) = (float)s;
      *(float*)(data.sic) = (float)i;
      ResponseStatus rs = e32ttl.sendFixedMessage(0, 7, 17, &data, sizeof(Signal));
      Serial.println(rs.getResponseDescription());
      delay(4000);
    }
  }
}

double sicak()
{
  sicaklik = bmp.readTemperature();
  return sicaklik;
}

double irtifa()
{
  basinc = bmp.readPressure();
  basinc /= 100;
  basincirtifa = 44330 * (1.0 - pow(basinc / denizbasinc, 0.1903));
  return basincirtifa;
}
