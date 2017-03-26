#include <Servo.h>

/*
    Steering Col
  1500 ms Straight
  2000 ms Left
  1000 ms Right

    
 */
#define STEERING_SERVO 11
#define MOTOR_ESC 10

Servo steering;
Servo motor;

void setup() {
  steering.attach(STEERING_SERVO);
  motor.attach(MOTOR_ESC);
  // Set midpoints
  steering.writeMicroseconds(1500);
  motor.writeMicroseconds(1500);
}

void loop() {

}
