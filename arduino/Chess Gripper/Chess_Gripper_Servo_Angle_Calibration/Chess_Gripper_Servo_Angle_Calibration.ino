#include <Servo.h>

Servo servo;

void setup() {
  servo.attach(9);
}

void loop() {
  servo.write(25);
  delay(3000);
  servo.write(120);
  delay(3000);
}
