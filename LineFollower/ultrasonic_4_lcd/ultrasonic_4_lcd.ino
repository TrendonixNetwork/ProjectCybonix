/*


HC-SR04 Ultrasonic Sensor

  VCC to Arduino 5V
  GND to Arduino GND
  Echo to Arduino pin 12
  Trig to Arduino pin 13

LCD Display
 
  VSS to Arduino GND
  VCC to Arduino 5V
  VEE to Arduino GND
  RS to Arduino pin 11
  R/W to Arduino pin 10
  E to Arduino pin 9
  DB4 to Arduino pin 3
  DB5 to Arduino pin 4
  DB6 to Arduino pin 5
  DB7 to Arduino pin 6
  LED+ to Arduino 5V
  LED- to Arduino GND
  

*/
#include <LiquidCrystal.h> //Load Liquid Crystal Library
LiquidCrystal LCD(11,10,9,3,4,5,6);  //Create Liquid Crystal Object called LCD

#define trig1 24 //Sensor Echo pin connected to Arduino pin 13
#define echo1 25 //Sensor Trip pin connected to Arduino pin 12
#define trig2 28
#define echo2 29
#define trig3 32
#define echo3 33
#define trig4 36
#define echo4 37

#define pin1 13

void setup() 
{  
  Serial.begin(9600);
  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(trig3, OUTPUT);
  pinMode(echo3, INPUT);
  pinMode(trig4, OUTPUT);
  pinMode(echo4, INPUT);
    
  LCD.begin(16,2); //Tell Arduino to start your 16 column 2 row LCD
  LCD.setCursor(0,0);  //Set LCD cursor to upper left corner, column 0, row 0
  LCD.print("Target Distance:");  //Print Message on First Row

}

void loop() 
{

  
 sensor1();
 delay(1000);
 sensor2();
 delay(1000);
 sensor3();
 delay(1000);
 sensor4();
delay(1000);

  


}
void sensor1()
{
  int dist1,dur1;
  digitalWrite(trig1, LOW);
  
  delayMicroseconds(2);
  digitalWrite(trig1, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(trig1, LOW);
 
  
  dur1 = pulseIn(echo1, HIGH);
  dist1 = (dur1/2) / 29.1;
  Serial.print("distance 1 =");
Serial.println(dist1);
if(dist1 >=50)
{
  digitalWrite(pin1,HIGH);
}
else
{
  digitalWrite(pin1,LOW);
}
  
  
}
void sensor2()
{
 int dist2,dur2;
  digitalWrite(trig2, LOW);
  
  delayMicroseconds(2);
  
  digitalWrite(trig2, HIGH);
  
  delayMicroseconds(10);
  
  digitalWrite(trig2, LOW);
  
  dur2 = pulseIn(echo2, HIGH);
  dist2 = (dur2/2) / 29.1;
  Serial.print("distance 2 =");
  Serial.println(dist2);
  if(dist2 >=50)
{
  digitalWrite(pin1,HIGH);
}
else
{
  digitalWrite(pin1,LOW);
}
  
  
}

void sensor3()
{
  int dist3,dur3;
  digitalWrite(trig3, LOW);
  
  delayMicroseconds(2);
  
  digitalWrite(trig3, HIGH);
  
  delayMicroseconds(10);
  digitalWrite(trig3, LOW);
  
  
  dur3 = pulseIn(echo3, HIGH);
  dist3 = (dur3/2) / 29.1;
  Serial.print("distance 3 =");
  Serial.println(dist3);
  if(dist3 >=50)
{
  digitalWrite(pin1,HIGH);
}
else
{
  digitalWrite(pin1,LOW);
}
  
  
}

void sensor4()
{
  int dist4,dur4;
  digitalWrite(trig4, LOW);
  delayMicroseconds(2);
  digitalWrite(trig4, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig4, LOW);
  
  dur4 = pulseIn(echo4, HIGH);
  dist4 = (dur4/2) / 29.1;
  Serial.print("distance 4 =");
Serial.println(dist4);
if(dist4 >=50)
{
  digitalWrite(pin1,HIGH);
}
else
{
  digitalWrite(pin1,LOW);
}
  
  
}

  


