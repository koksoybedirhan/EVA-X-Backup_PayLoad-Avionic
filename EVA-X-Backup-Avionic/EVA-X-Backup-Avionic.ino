//Kütüphaneler
#include <Wire.h>
#include <SFE_BMP180.h>
#include "SparkFunBME280.h" 
#include <SimpleKalmanFilter.h>
#include "TinyGPS++.h"

//Tanımlamalar
BME280 bme280;
SFE_BMP180 bmp180;
SimpleKalmanFilter Kalmanbasinci(1, 1, 0.01);
TinyGPSPlus gps;

//Pinler
int buzzerPin = 42; //Mega Pro'ya göre ayarlanacak
int valf1 = 29; //Mega Pro'ya göre ayarlanacak
int valf2 = 30; //Mega Pro'ya göre ayarlanacak

//Tanımlar
char durum;
int aktiflikDurumu = 1;
double T, bmp180basinc, bme280basinc, bmpmaks = 0;
double bmp180convert, bmpkalmanolculenbasinc, bmekalmanolculenbasinc;
int pos = 0; 
double bmpbasincIrtifa, bmebasincdegeri;
double bmpIrtifaDegeri, bmeIrtifaDegeri, Irtifafonk, denizbasinci = 966.6;
bool birinciayrilma = false;
double bmekalm, bmpkalm;
int eskiZaman1 = 0, ayrilmaEskiZaman = 0;
int gpsZaman, ayrilmaYeniZaman;
int eskiZaman2 = 0;
unsigned long dorjiZaman;
int eskiZaman3 = 0;
int zamanOlcme1 = 0;
int zamanOlcme2 = 0;
unsigned long irtifaZaman;
double latitude,longtitude, altitude;

//Telemetri Tanımlamaları
char dorjiAdres[4] = "EVA", latchar[10], altchar[7], longchar[10], bmpchar[7], bmechar[7];
String dorjiGonderim = "EVA", bosluk = ",", durumstring;

void setup()
{
  Serial.begin(9600);
  Serial2.begin(9600);
  Wire.begin();
  bme280.setI2CAddress(0x76); 
  pinMode(13, OUTPUT); // led yakarak hata bakma
  pinMode(valf1, OUTPUT); //valf1
  pinMode(valf2, OUTPUT); //valf2
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

  durumstring = String(aktiflikDurumu);
  
  //Serial.println("Kalman filtresi uygulandı.");

  bmpbasincIrtifa = bmp180fonk();
}

void loop()
{ 
  while(Serial2.available())//Yedek aviyonik için en önemli isteri GPS verisi olduğu için ilk olarak GPS'nin kesin çalıştığından emin olunacak. 
  {//Yapılan testler sonucunda GPS'in ortak çalışamadığı gözlemlenmiştir.
    if(gps.encode(Serial2.read()))
    { 
    gpsZaman = millis(); 
    dorjiZaman = millis(); 
    irtifaZaman = millis(); 
    if(gpsZaman-eskiZaman1 > 1000)
    {  
      latitude = gps.location.lat(), 6;
      dtostrf(latitude, 9, 6, latchar);
      longtitude = gps.location.lng(), 6;
      dtostrf(longtitude, 9, 6, longchar);
      altitude = gps.altitude.meters(), 6;
      dtostrf(altitude, 6, 1, altchar);
      //GpsEncode();
      eskiZaman1 = gpsZaman;
    }
    if(dorjiZaman-eskiZaman2 > 1000)
    {
      dorjifonk(); //dorji seri porttan aldığı veriyi direkt gönderiyor
      eskiZaman2 = dorjiZaman;
      buzzerHigh();
    }
    if(irtifaZaman-eskiZaman3 > 1000)
    {
      bmpkalm = bmpkalman();
      bmekalm = bmekalman();
      dtostrf(bmpkalm, 6, 1, bmpchar);
      dtostrf(bmekalm, 6, 1, bmechar);
      if(bmpkalm>bmpmaks)
      {
        bmpkalm = bmpmaks;
      }
      /*Serial.print("BMP180 Kalman İrtifa: ");
      Serial.print(bmpkalm, 1);
      Serial.print(" metre ");
      Serial.print("BME280 Kalman İrtifa: ");
      Serial.print(bmekalm, 1);
      Serial.println(" metre ");*/
      ayrilmafonk();
      
      eskiZaman3 = irtifaZaman;
    }
    
    zamanOlcme1 = millis();
    aktiflik3();
    zamanOlcme2 = millis();
    }
  }
}

void ayrilmafonk()
{
  ayrilmayeniZaman = millis();
  if(bmpkalm-bmpmaks==50 && bmekalm >= 3000 && birinciayrilma == false) //birinci ayrılma
  {//Bir sensör maks irtifaya ulaşıldığını hesaplayarak ayrılma yaparken diğeri 3000 metre ile ayrılma yapacak.
    delay(5000); //Birinci ayrılma için 5 saniye bekleniyor. Eğer 5 saniye sonra hala ayrılma olmazsa ayrılma gerçekleşecek.
    if(bmpkalm-bmpmaks==50 && bmekalm >= 3000)
    {
      digitalWrite(valf1, HIGH); //2900 metreye kadar açık kalacak
      Serial.println("First Seperation Done.");
      birinciayrilma = true;
      aktiflikDurumu = 2;
      buzzerHigh(); //2900 metreye kadar buzzer ötecek
    }
  }
  else if(bmpkalm-bmpmaks==250 && bmekalm <= 2900 && birinciayrilma == true)
  { //100 metreden sonra aktiflikler kaldırılacak.
    digitalWrite(valf1, LOW);
    buzzerLow(); 
  }
  else if(zamanOlcme1 - zamanOlcme2 == 30000)
  {//30000 tahmini değer. Roketin paraşütsüz 3000 metre çıkıp 2000'e düşüşü hesaplanıp yazılacak.
    aktiflikDurumu = 3;
  }
  else if(bmpkalm <= 500 && bmekalm <= 500 && birinciayrilma == true)//ikinci ayrılma
  {
    digitalWrite(valf2, HIGH);
    aktiflikDurumu = 4;
    Serial.println("500 meters detected with pressure, Second Seperation done");
    buzzerHigh(); // 500 metreden sonra yere düşünce bulması kolaylaştırılacak.
  }
  else if(bmpkalm <= 300 && bmekalm <= 300 && birinciayrilma == true)
  {
    digitalWrite(valf2, LOW);
  }
}

void aktiflik3()
{
  if(bmpkalm <=2000 && bmekalm<=2000 && birinciayrilma == true)
  {
    digitalWrite(13, HIGH);
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
  durumstring = String(aktiflikDurumu);
  //Verilerin karışmaması için EVA ile başlamayan string'leri okumayacak.
  dorjiGonderim = "dorjiAdres + bosluk + latitude + bosluk + longtitude + bosluk + altitude + bosluk + bmpstring + bosluk + bmestring + aktiflik durumu";
  Serial.println(dorjiGonderim);
}
