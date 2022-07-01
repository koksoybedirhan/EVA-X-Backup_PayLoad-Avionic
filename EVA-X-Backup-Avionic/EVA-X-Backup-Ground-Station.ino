String okunan;
char son[50];

void setup() 
{
  Serial.begin(9600);
}

void loop() 
{
  if (Serial.available() > 0) 
  {    
    okunan = Serial.readString();
    okunan.toCharArray(son, 50);
    if(son[0] == "E" && son[1] == "V" && son[2] == "A") 
    {//Gelen Verilerin EVA X takımı tarafından geldiğinin kontrolü
      Serial.println(son);
    }
    else
    {
      Serial.println("Istenilen Veri Tipi Gelmedi");
    }
  }
}
