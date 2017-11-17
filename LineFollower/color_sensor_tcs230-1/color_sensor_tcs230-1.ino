#define S0 22
#define S1 23
#define S2 24
#define S3 25

#define lfr1 30
#define lfr2 31
#define lfr3 32
#define lfr4 33

#define sensorOut 26

#define pin1 13

#include<tcssensor.h>
int frequency1 = 0;
int frequency2 = 0;
int frequency3 = 0;

void setup()
{
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(lfr1, OUTPUT);
  pinMode(lfr2, OUTPUT);
  pinMode(lfr3, OUTPUT);
  pinMode(lfr4, OUTPUT);
  pinMode(pin1, OUTPUT);
  pinMode(sensorOut, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S0,HIGH);
  digitalWrite(S1,LOW);
  
  Serial.begin(9600);
}
void loop()
{
  // Setting red filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,LOW);
  
  // Reading the output frequency
  frequency1 = pulseIn(sensorOut, LOW);
  
  // Printing the value on the serial monitor
  Serial.print("R= ");//printing name
  Serial.print(frequency1);//printing RED color frequency
  Serial.print("  ");
  delay(1000);
  
  // Setting Green filtered photodiodes to be read
  digitalWrite(S2,HIGH);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  frequency2 = pulseIn(sensorOut, LOW);
  
  // Printing the value on the serial monitor
  Serial.print("G= ");//printing name
  Serial.print(frequency2);//printing RED color frequency
  Serial.print("  ");
  delay(1000);
  
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S2,LOW);
  digitalWrite(S3,HIGH);
  
  // Reading the output frequency
  frequency3 = pulseIn(sensorOut, LOW);
  // Printing the value on the serial monitor
  Serial.print("B= ");//printing name
  Serial.print(frequency3);//printing RED color frequency
  Serial.println("  ");
  delay(1000);

  if((frequency1<=1000 && frequency1>=10)&&(frequency2<=1000 && frequency2>=10) && (frequency3<=1000 && frequency3>=10))
  {
    Serial.println("working");
    digitalWrite(pin1,HIGH);
    
    digitalWrite(lfr1,LOW);
    digitalWrite(lfr2,HIGH);
    digitalWrite(lfr3,LOW);
    digitalWrite(lfr4,HIGH);
    delay(200);
  }
  else
  {
  digitalWrite(pin1,LOW);
    
    digitalWrite(lfr1,LOW);
    digitalWrite(lfr2,LOW);
    digitalWrite(lfr3,LOW);
    digitalWrite(lfr4,LOW);
  }
}
