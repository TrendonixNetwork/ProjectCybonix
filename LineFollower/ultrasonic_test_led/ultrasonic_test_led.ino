/*
HC-SR04 Ultrasonic Sensor with LCD dispaly

HC-SR04 Ultrasonic Sensor
  VCC to Arduino 5V
  GND to Arduino GND
  Echo to Arduino pin 12
  Trig to Arduino pin 13

LCD Display (I used JHD162A) 
  VSS to Arduino GND
  VCC to Arduino 5V
  VEE to Arduino GND
  RS to Arduino pin 11
  R/W to Arduino pin 10
  E to Arduino pin 9
  DB4 to Arduino pin 2
  DB5 to Arduino pin 3
  DB6 to Arduino pin 4
  DB7 to Arduino pin 5
  LED+ to Arduino 5V
  LED- to Arduino GND
  
Modified by Ahmed Djebali (June 1, 2015).
*/
#include <LiquidCrystal.h> //Load Liquid Crystal Library
LiquidCrystal LCD(11,10,9,40,41,42,43);  //Create Liquid Crystal Object called LCD

#define trigPin 22 //Sensor Echo pin connected to Arduino pin 13
#define echoPin 23 //Sensor Trip pin connected to Arduino pin 12
#define pin1 13
//Simple program just for testing the HC-SR04 Ultrasonic Sensor with LCD dispaly 
//URL:

void setup() 
{  
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  LCD.begin(16,2); //Tell Arduino to start your 16 column 2 row LCD
  LCD.setCursor(0,0);  //Set LCD cursor to upper left corner, column 0, row 0
  LCD.print("Target Distance:");  //Print Message on First Row
}

void loop()
{
  long duration, distance;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  
if(distance >=50)
{
  digitalWrite(pin1,HIGH);
}
else
{
  digitalWrite(pin1,LOW);
}

  LCD.setCursor(0,1);  //Set cursor to first column of second row
  Serial.println("                "); //Print blanks to clear the row
  LCD.setCursor(0,1);   //Set Cursor again to first column of second row
  Serial.print(distance); //Print measured distance
  Serial.print(" cm");  //Print your units.
  delay(250); //pause to let things settle
  
}

