#include <Servo.h>
#define PIN_SERVO 10

Servo myservo;

void setup() {
  myservo.attach(PIN_SERVO); 
  myservo.write(90);
  delay(1000);
}

void loop() {
  while(1) {
    myservo.write(0);
    delay(500);
    myservo.write(15);
    delay(500);
    myservo.write(30);
    delay(500);
    myservo.write(45);
    delay(500);
    myservo.write(60) ;
    delay(500);
    myservo.write(45) ;
    delay(500);
    myservo.write(30) ;
    delay(500);
    myservo.write(15) ;
    delay(500);
    myservo.write(0);
    delay(500); 
  }
}
