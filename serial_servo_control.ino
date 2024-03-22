#include <Servo.h>

Servo servo1;
Servo servo2;


void setup() {
  Serial.begin(9600);
  servo1.attach(2);
  servo2.attach(3);
   servo1.write(0);
    servo2.write(0);
   
}

void loop() {
  if (Serial.available() >= 4) {
    int servo1_pos = Serial.parseInt();
    int servo2_pos = Serial.parseInt();

    servo1.write(servo1_pos);
    servo2.write(servo2_pos);

    Serial.println(servo1_pos);
  }
}
