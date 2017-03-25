#include <NewPing.h>

/*
  Echo and trigger are digital viewers. Sensor has an apparent range of
  400cm, buuuut, cutting that in half to hopefully reduce error.
 */

#define US_ECHO 3
#define US_TRIGGER 2
#define US_MAX_ECHO_CM_DISTANCE 200  //

NewPing sonar(US_TRIGGER, US_ECHO, US_MAX_ECHO_CM_DISTANCE);

void setup() {
  Serial.begin(9600);
}

void loop() {
  delay(50);
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm());
  Serial.println("cm");
}
