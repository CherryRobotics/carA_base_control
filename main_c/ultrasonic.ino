#include <NewPing.h>

#define US_ECHO d2
#define US_TRIGGER d3
#define US_MAX_ECHO_CM_DISTANCE 90  // 3ft

NewPing sonar(US_TRIGGER, US_ECHO, US_MAX_ECHO_CM_DISTANCE);

void setup() {
  Serial.begin(115200);
}

void loop() {
  delay(50);
  Serial.print("Ping: ")
  Serial.print(sonar.ping_cm());
  Serial.println("cm");
}
