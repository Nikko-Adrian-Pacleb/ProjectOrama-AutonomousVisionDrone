#include <Servo.h>

Servo ESC; 

int Speed; 

void setup(){
ESC.attach(10);
ESC.writeMicroseconds(1000);
ESC.write(180);
ESC.write(0);
}

void loop(){
  Speed = analogRead(A0);
  Speed = map(Speed, 0, 1023, 0, 180);
  ESC.write(90);
}