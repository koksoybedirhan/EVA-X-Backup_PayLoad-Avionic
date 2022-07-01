//kütüphaneler
#include <Wire.h>
#include <SFE_BMP180.h>
#include "SparkFunBME280.h" 
#include "ServoTimer2.h"
#include <SimpleKalmanFilter.h>
#include "TinyGPS++.h"

//tanımlamalar
BME280 bme280;
SFE_BMP180 bmp180;
ServoTimer2 servoM1;
ServoTimer2 servoM2;
SimpleKalmanFilter Kalmanbasinci(1, 1, 0.01);
TinyGPSPlus gps;

//tanımlar
char durum;
double T, bmp180basinc, bme280basinc;
double bmp180convert, bmpkalmanolculenbasinc, bmekalmanolculenbasinc;
int pos = 0; 
double bmpbasincIrtifa, bmebasincdegeri;
double bmpIrtifaDegeri, bmeIrtifaDegeri, Irtifafonk, denizbasinci = 966.6;
bool birinciayrilma = false;
double bmekalm, bmpkalm;
int buzzerPin = 7; //Mega Pro'ya göre ayarlanacak
int eskiZaman1 = 0;
int gpsZaman;
int eskiZaman2 = 0;
unsigned long dorjiZaman;
int eskiZaman3 = 0;
unsigned long irtifaZaman;
double latitude,longtitude, altitude;
char dorjiAdres[4] = "EVA", latchar[10], altchar[7], longchar[10], bmpchar[7], bmechar[7];
String dorjiGonderim = "EVA";

void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600);
  Wire.begin();
  bme280.setI2CAddress(0x76); 
  servoM1.attach(9);
  servoM2.attach(10);
  pinMode(13, OUTPUT); // led yakarak hata bakma
  //Serial.print("Sensörler Başlatılıyor ");
  
  if(bme280.beginI2C() == false) 
  {
    //Serial.print("BME280 başlatılmadı. ");
    pinMode(13, HIGH);
  }
  else
  {
    //Serial.println("BME280 başlatıldı.");
    pinMode(13, LOW);
  }
 
  if (bmp180.begin() == false) 
  {
    //Serial.println("BMP180 başlatılamadı."); 
    pinMode(13, HIGH);
  }
  else
  {
    //Serial.println("BMP180 başlatıldı."); 
    pinMode(13, LOW);
  }

  //Serial.println("Kalman filtresi uygulandı.");

  bmpbasincIrtifa = bmp180fonk();
}

void loop()
{  
  gpsZaman = millis(); 
  dorjiZaman = millis(); 
  irtifaZaman = millis(); 
  if(gpsZaman-eskiZaman1 > 1000)
  {  
    if(gps.encode(Serial2.read()))
    {
      latitude = gps.location.lat(), 6;
      dtostrf(latitude, 9, 6, latchar);
      longtitude = gps.location.lng(), 6;
      dtostrf(longtitude, 9, 6, longchar);
      altitude = gps.altitude.meters(), 6;
      dtostrf(altitude, 6, 1, altchar);
      //GpsEncode();
    }
    eskiZaman1 = gpsZaman;
  }
  if(dorjiZaman-eskiZaman2 > 1000)
  {
    dorjifonk(); //dorji seri porttan aldığı veriyi direkt gönderiyor
    eskiZaman2 = dorjiZaman;
  }
  if(irtifaZaman-eskiZaman3 > 1000)
  {
    bmpkalm = bmpkalman();
    bmekalm = bmekalman();
    dtostrf(bmpkalm, 6, 1, bmpchar);
    dtostrf(bmekalm, 6, 1, bmechar);
    /*Serial.print("BMP180 Kalman İrtifa: ");
    Serial.print(bmpkalm, 1);
    Serial.print(" metre ");
    Serial.print("BME280 Kalman İrtifa: ");
    Serial.print(bmekalm, 1);
    Serial.println(" metre ");*/
    ayrilmafonk();
    
    eskiZaman3 = irtifaZaman;
  }
}

void ayrilmafonk()
{
  if(bmpkalm >= 3000 && bmekalm >= 3000 && birinciayrilma == false) //birinci ayrılma
  {
    servofonk1();
    Serial.println("First Seperation Done.");
    birinciayrilma = true;
    buzzerHigh();
  }
  else if(bmpkalm <= 500 && bmekalm <= 500 && birinciayrilma == true)//ikinci ayrılma
  {
    servofonk2();
    Serial.println("500 meters detected with pressure, Second Seperation done");
    buzzerLow();
  }
}

double bmpkalman()
{
  bmp180basinc = bmp180fonk();
  bmpIrtifaDegeri = bmp180.altitude(bmp180basinc,bmpbasincIrtifa);
  bmpkalmanolculenbasinc = Kalmanbasinci.updateEstimate(bmpIrtifaDegeri);
  return bmpkalmanolculenbasinc;
}

double bmekalman()
{
  bmeIrtifaDegeri = bme280fonk();
  bmekalmanolculenbasinc = Kalmanbasinci.updateEstimate(bmeIrtifaDegeri);
  return bmekalmanolculenbasinc;
}

double bmp180fonk()
{
  durum = bmp180.startTemperature();
  if (durum != 0) {
    delay(1000);
    durum = bmp180.getTemperature(T);
    if (durum != 0) {
      durum = bmp180.startPressure(3);
      if (durum != 0) {
        delay(durum);
        durum = bmp180.getPressure(bmp180basinc, T);
        if (durum != 0) {
          //Serial.print("Basınç: ");
          //Serial.print(P);
          //Serial.println(" hPa");
        }
      }
    }
  }
  return (bmp180basinc);
}

double bme280fonk()
{
  bmebasincdegeri = bme280.readFloatPressure()/100, 2;
  Irtifafonk = 44330*(1.0-pow(bmebasincdegeri/denizbasinci,0.1903));
  return(Irtifafonk);
}

void servofonk1()
{
  for (pos = 0; pos <= 90; pos += 1) 
  {                
    servoM1.write(pos);              
    delay(15);                       
  }
  for (pos = 90; pos >= 0; pos -= 1) { 
    servoM1.write(pos);             
    delay(15);                      
  }
}

void servofonk2()
{
  for (pos = 0; pos <= 90; pos += 1) 
  {                
    servoM2.write(pos);              
    delay(15);                       
  }
  for (pos = 90; pos >= 0; pos -= 1) { 
    servoM2.write(pos);             
    delay(15);                      
  }
}

void buzzerHigh()
{
  digitalWrite(buzzerPin, HIGH);
  Serial.println("Buzzer Aktif Edildi");
}

void buzzerLow()
{
  digitalWrite(buzzerPin, LOW);
  Serial.println("Buzzer Aktif Edilmedi");
}

/*void GpsEncode()
{
  String msg = Serial2.readStringUntil('\r');
  Serial.print("LAT="); Serial.println(gps.location.lat(), 6);
  Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
  Serial.print("ALT="); Serial.println(gps.altitude.meters(), 6);
}*/    

void dorjifonk()
{
  //Verilerin karışmaması için EVA ile başlamayan string'leri okumayacak.
  dorjiGonderim = "dorjiAdres + latitude + longtitude + altitude + bmpstring + bmestring";
  Serial.println(dorjiGonderim);
}
