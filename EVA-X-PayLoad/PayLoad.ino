//kutuphaneler
#include <Wire.h>
#include "TinyGPS++.h"
#include "SparkFunBME280.h"
#include "LoRa_E32.h"
#include <SimpleKalmanFilter.h>
BME280 bme; //Uses I2C address 0x76 (jumper closed)
SimpleKalmanFilter KalmanIrtifa(1, 1, 0.01);
TinyGPSPlus gps;
SoftwareSerial portLora(19, 18);
LoRa_E32 e32ttl(&portLora);

//tanimlamalar
float bmebasinc, irtifa, kalman, nem, sicaklik;
float denizbasinc = 966.6;

typedef struct {
byte altitude[5];
byte latitude[10];
byte longtitude[10];
//byte kalman[7];
//byte sicaklik[5];
//byte nem[5];
} Signal;
Signal data;

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  /*Basınç, sıcaklık ve nem hakem tarafından istenirse kullanılacak.
  Wire.begin();
  bme.setI2CAddress(0x76); //Connect to a second sensor

  if(bme.beginI2C() == true)
  {
    Serial.println("BME connected");
  }
  if(bme.beginI2C() == false) 
  {
    Serial.println("BME not connected");
  }*/
}

void loop() 
{
  while(Serial2.available())
  {
    if(gps.encode(Serial2.read()))
    {
      String GPSokunan = Serial2.readStringUntil('\r');
      Serial.println(GPSokunan);
      Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
      *(float*)(data.latitude) = (gps.location.lat());
      Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
      *(float*)(data.longtitude) = (gps.location.lng());
      Serial.print("ALT="); Serial.println(gps.altitude.meters(), 1);
      *(float*)(data.altitude) = (gps.altitude.meters());
      delay(4*1000); //Sure denenecek.
    }
    ResponseStatus rs = e32ttl.sendFixedMessage(0, 14, 29, &data, sizeof(Signal));
    Serial.println(rs.getResponseDescription());
    delay(650);
  }
}

/* Şartnamede istenilen ama hyi'de istenilmeyen veriler.
void irtifaFonk()
{
  bmebasinc = bme.readFloatPressure()/101325, 4;
  irtifa = 44330*(1.0-pow(bmebasinc/denizbasinc,0.1903));
  kalman = KalmanIrtifa.updateEstimate(irtifa);
  return kalman;
}

void nemFonk()
{
  nem = bme.readFloatHumidity(), 0);
  return nem;
}

void sicaklikFonk()
{
  sicaklik = bme.readTempC(), 2);
  return sicaklik;
}
*/
