//#define RH_ENCODER_A 3 
//#define RH_ENCODER_B 5

#define encoderA 4
#define encoderB 5

//#define E1 10  // Enable Pin for motor 1

 
#define in2 3  // Control pin 1 for motor 1
#define in1 2  // Control pin 2 for motor 1

 
// variables to store the number of encoder pulses
// for each motor
volatile unsigned long leftCount = 0;
//volatile unsigned long rightCount = 0;
 
void setup() {
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  //pinMode(E1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in1, OUTPUT);
    
 // pinMode(RH_ENCODER_A, INPUT);
  //pinMode(RH_ENCODER_B, INPUT);
  
  // initialize hardware interrupts
 
 // attachInterrupt(1, rightEncoderEvent, CHANGE);
  
  Serial.begin(9600);
}
 
void loop() 
{
 // Serial.print("Right Count: ");
  //Serial.println(rightCount);
  
  leftEncoderEvent();
  Serial.print("Left Count: ");
  Serial.println(leftCount);
  Serial.println();
  delay(500);
}
 
// encoder event for the interrupt call
void leftEncoderEvent()
{
  if (digitalRead(encoderA) == HIGH) {
    if (digitalRead(encoderB) == LOW) {
      
      //analogWrite(E1, 255);  // Run in full speed
    
 
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    leftCount++;
    Serial.println("high low");
    }
    else
    {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    Serial.println("nothin1"); 
  } 
  }
   else if (digitalRead(encoderA) == LOW) {
    if (digitalRead(encoderB) == HIGH) {
      
      //analogWrite(E1, 255);  // Run in full speed
    
 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    leftCount++;
    Serial.println("high high");
    }
    else
    {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    Serial.println("nothin"); 
  }  
  } 
}
  
  /* else if (digitalRead(encoderA) == LOW) {
    if (digitalRead(encoderB) == HIGH) {
      
      //analogWrite(E1, 255);  // Run in full speed
    
 
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    leftCount++;
    Serial.println("low high");
    } 
    else
    {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    Serial.println("nothin"); 
  } 
  } 
  
  
  else if (digitalRead(encoderA) == LOW) {
    if (digitalRead(encoderB) == LOW) {
      
      //analogWrite(E1, 255);  // Run in full speed
    
 
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    leftCount++;
    Serial.println("low low");
    }
    else
    {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    Serial.println("nothin"); 
  }  
  } 
  
}
 
// encoder event for the interrupt call
/*void rightEncoderEvent() {
  if (digitalRead(RH_ENCODER_A) == HIGH) {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount++;
    } else {
      rightCount--;
    }
  } else {
    if (digitalRead(RH_ENCODER_B) == LOW) {
      rightCount--;
    } else {
      rightCount++;
    }
  }
}*/
