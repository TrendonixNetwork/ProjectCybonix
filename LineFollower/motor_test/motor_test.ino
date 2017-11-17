#define mot1g 2  // Control pin 1 for motor 1
#define mot1y 3  // Control pin 2 for motor 1

#define in1 6
#define in2 7

#define mot2g 4  // Control pin 1 for motor 2
#define mot2y 5  // Control pin 2 for motor 2

#define in3 8
#define in4 9

#define Ea 10
#define Eb 11

#define S10 22
#define S11 23
#define S12 24
#define S13 25
#define sensorOut1 26

#define S20 27
#define S21 28
#define S22 29
#define S23 30
#define sensorOut2 31

#define S30 32
#define S31 33
#define S32 34
#define S33 35
#define sensorOut3 36

int frequency1 = 0;
int frequency2 = 0;
int frequency3 = 0;
int v1=0;
int v2=0;
int v3=0;
int v4=0;

void setup() {
 
    pinMode(Ea, OUTPUT);
    pinMode(Eb, OUTPUT);
 
    pinMode(mot1g, INPUT);
    pinMode(mot1y, INPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(mot2g, INPUT);
    pinMode(mot2y, INPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

  pinMode(S10, OUTPUT);
  pinMode(S11, OUTPUT);
  pinMode(S12, OUTPUT);
  pinMode(S13, OUTPUT);
  pinMode(sensorOut1, INPUT);

  pinMode(S20, OUTPUT);
  pinMode(S21, OUTPUT);
  pinMode(S22, OUTPUT);
  pinMode(S23, OUTPUT);
  pinMode(sensorOut2, INPUT);

  pinMode(S30, OUTPUT);
  pinMode(S31, OUTPUT);
  pinMode(S32, OUTPUT);
  pinMode(S33, OUTPUT);
  pinMode(sensorOut3, INPUT);
  
  // Setting frequency-scaling to 20%
  digitalWrite(S10,HIGH);
  digitalWrite(S11,LOW);

  digitalWrite(S20,HIGH);
  digitalWrite(S21,LOW);

  digitalWrite(S30,HIGH);
  digitalWrite(S31,LOW);
    
    Serial.begin(9600);
    
    v1=digitalRead(mot1g);
    v2=digitalRead(mot1y);
    v3=digitalRead(mot2g);
    v4=digitalRead(mot2y);
    
}
 
void loop()
{
  
  digitalWrite(S12,LOW);
  digitalWrite(S13,LOW);

  digitalWrite(S22,LOW);
  digitalWrite(S23,LOW);

  digitalWrite(S32,LOW);
  digitalWrite(S33,LOW);
  
  // Reading the output frequency
  frequency1 = pulseIn(sensorOut1, LOW);
  frequency2 = pulseIn(sensorOut2, LOW);
  frequency3 = pulseIn(sensorOut3, LOW);
  
  // Printing the value on the serial monitor
  Serial.print("R= ");//printing name
  Serial.println(frequency1);//printing RED color frequency
  Serial.print("  ");
  Serial.println(frequency2);//printing RED color frequency
  Serial.print("  ");
  Serial.println(frequency3);//printing RED color frequency
  Serial.print("  ");
  delay(1000);
  
  // Setting Green filtered photodiodes to be read
  digitalWrite(S12,HIGH);
  digitalWrite(S13,HIGH);

  digitalWrite(S22,HIGH);
  digitalWrite(S23,HIGH);

  digitalWrite(S32,HIGH);
  digitalWrite(S33,HIGH);
  
  // Reading the output frequency
  frequency1 = pulseIn(sensorOut1, LOW);
  frequency2 = pulseIn(sensorOut2, LOW);
  frequency3 = pulseIn(sensorOut3, LOW);
  
  // Printing the value on the serial monitor
  Serial.print("G= ");//printing name
  Serial.println(frequency1);//printing RED color frequency
  Serial.print("  ");
  Serial.println(frequency2);//printing RED color frequency
  Serial.print("  ");
  Serial.println(frequency3);//printing RED color frequency
  Serial.print("  ");
  delay(1000);
  
  // Setting Blue filtered photodiodes to be read
  digitalWrite(S12,LOW);
  digitalWrite(S13,HIGH);

   digitalWrite(S22,LOW);
  digitalWrite(S23,HIGH);

   digitalWrite(S32,LOW);
  digitalWrite(S33,HIGH);
  
  // Reading the output frequency
  frequency1 = pulseIn(sensorOut1, LOW);
  frequency2 = pulseIn(sensorOut2, LOW);
  frequency3 = pulseIn(sensorOut3, LOW);
  
  // Printing the value on the serial monitor
  Serial.print("B= ");//printing name
  Serial.println(frequency1);//printing RED color frequency
  Serial.println("  ");
  Serial.println(frequency2);//printing RED color frequency
  Serial.println("  ");
  Serial.println(frequency3);//printing RED color frequency
  Serial.println("  ");
  delay(1000);

// if(frequency1
    Serial.println("delay");
    delay(2000);
    analogWrite(Ea,100); // Run in half speed
    analogWrite(Eb,100);
    Serial.println("speed = 50");
    
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    Serial.println("logic hai");
    
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    Serial.println("logic yaha bhi  hai");
    
    Serial.print("v1 : ");
    Serial.println(v1);
    Serial.print("v2 : ");
    Serial.println(v2);

    Serial.print("v3 : ");
    Serial.println(v3);
    Serial.print("v4 : ");
    Serial.println(v4);
    delay(2000);
 
    // change direction
 
    digitalWrite(Ea,LOW);
    digitalWrite(Eb,LOW);
    Serial.println("speed = 0");
 
  delay(3000);
 
    analogWrite(Ea,100);  // Run in full speed
    analogWrite(Eb,100);
    Serial.println("speed = 50");
    
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    Serial.println("logic hai");
    
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    Serial.println("logic yaha bhi hai");
    
    Serial.print("v1 : ");
    Serial.println(v1);
    
    Serial.print("v2 : ");
    Serial.println(v2);
    
    Serial.print("v3 : ");
    Serial.println(v3);
    
    Serial.print("v4 : ");
    Serial.println(v4);
 
    delay(2000);
    
}
