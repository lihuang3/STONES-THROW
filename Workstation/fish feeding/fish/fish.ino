#include <Servo.h>

Servo myservo;  // create servo object to control a servo

byte hour = 0; 
byte minu = 0; 
byte cnt = 0;
byte flag = 0;
int potpin = 0;  // analog pin used to connect the potentiometer
int val;    // variable to read the value from the analog pin
void setup() {
  // put your setup code here, to run once:
 myservo.attach(8); 
Serial.begin(9600);
while(!Serial){
  ;
}
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()>0){
    while(Serial.available()<2);

    hour = Serial.read();

    minu= Serial.read();
    
 
    Serial.write(hour);
    Serial.write(minu);
   // feed 3 times
    while(flag <3){
    //fixing water and food
    while(cnt < 45){
    myservo.write(0);
    delay(100);
    myservo.write(15);
    delay(100);
    cnt +=1;
    }
    cnt =0;

    //feeding
    myservo.write(175);
    delay(1200);
    myservo.write(70);
    delay(2000);
    flag +=1;
  }
   flag = 0;
  }
 
  
}


