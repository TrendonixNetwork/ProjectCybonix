#define dir1 2
#define pwm1 3
#define led 13
void setup() {
  
Serial.begin(9600);
pinMode(dir1,OUTPUT);
pinMode(pwm1,OUTPUT);
pinMode(led,OUTPUT);
}

void loop() 
{
  
digitalWrite(dir1,HIGH);
digitalWrite(pwm1,LOW);  
digitalWrite(led,HIGH);
Serial.println("working");
}
