#define lfr1 6
#define lfr2 7
#define lfr3 8
#define lfr4 9

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
 pinMode(lfr1, OUTPUT);
  pinMode(lfr2, OUTPUT);
  pinMode(lfr3, OUTPUT);
  pinMode(lfr4, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(lfr1,LOW);
    digitalWrite(lfr2,HIGH);
    digitalWrite(lfr3,LOW);
    digitalWrite(lfr4,HIGH);
    Serial.println("k");
}
