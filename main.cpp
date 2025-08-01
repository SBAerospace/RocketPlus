#include <PWMServo.h>


PWMServo myservo;
int pos = 0;


void setup() {
 myservo.attach(7);     // attach on pin 7 (50 Hz PWM by default)
}


void loop() {
 // extend to 45°
 for (pos = 0; pos <= 45; pos++) {
   myservo.write(pos);
   delay(15);
 }
 // hold at 45°
 delay(60000);
 // retract to 0°
 for (pos = 45; pos >= 0; pos--) {
   myservo.write(pos);
   delay(15);
 }
 // hold at 0°
 delay(1000);
}
