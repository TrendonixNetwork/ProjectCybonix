#define mot1g 2  // Control pin 1 for motor 1
#define mot1y 3  // Control pin 2 for motor 1

#define in1 6
#define in2 7

#define mot2g 4  // Control pin 1 for motor 2
#define mot2y 5  // Control pin 2 for motor 2

#define in3 8
#define in4 9

//#define Ea 10
//#define Eb 11

int v1=0;
int v2=0;
int v3=0;
int v4=0;

void setup() {
  // put your setup code here, to run once:

    //pinMode(Ea, OUTPUT);
    //pinMode(Eb, OUTPUT);
 
    pinMode(mot1g, INPUT);
    pinMode(mot1y, INPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    pinMode(mot2g, INPUT);
    pinMode(mot2y, INPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);

    v1=digitalRead(mot1g);
    v2=digitalRead(mot1y);
    v3=digitalRead(mot2g);
    v4=digitalRead(mot2y);
    
    
     Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
 Serial.println("delay");
    delay(1000);
    //analogWrite(Ea,255); // Run in half speed
    //analogWrite(Eb,255);
    Serial.println("speed = 100");
    
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    Serial.println("logic hai");
    
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    Serial.println("logic yaha bhi  hai");
    
    Serial.print("v1 : ");
    Serial.println(v1);
    Serial.print("v2 : ");
    Serial.println(v2);

    Serial.print("v3 : ");
    Serial.println(v3);
    Serial.print("v4 : ");
    Serial.println(v4);
    delay(1500);
 
    // change direction
 
   // digitalWrite(Ea,LOW);
    //digitalWrite(Eb,LOW);
    Serial.println("speed = 0");
 
  //delay(1000);
 
   // analogWrite(Ea,255);  // Run in full speed
   // analogWrite(Eb,255);
    Serial.println("speed = 100");
    
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    Serial.println("logic hai");
    
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    Serial.println("logic yaha bhi hai");
    
    Serial.print("v1 : ");
    Serial.println(v1);
    
    Serial.print("v2 : ");
    Serial.println(v2);
    
    Serial.print("v3 : ");
    Serial.println(v3);
    
    Serial.print("v4 : ");
    Serial.println(v4);
 
    delay(1500);
   // digitalWrite(Ea,LOW);
    //digitalWrite(Eb,LOW);
    Serial.println("speed = 0");
    
}

