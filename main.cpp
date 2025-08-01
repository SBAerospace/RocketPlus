#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "I2Cdev.h"
#include "Adafruit_MPU6050.h"
#include <Servo.h>
#include <math.h>


#define PIN 1 //TODO
#define CLOSED_ANGLE 1 //TODO
#define DEPLOYED_ANGLE 1 //TODO


#define SEALEVEL_PRESSURE_HPA (1013.25) // Update with actual pressure
#define AIR_DENSITY 1.225  // kg/m^3 Update if needed
#define TARGET_APOGEE_M 228.6


// Constants for normal conditions
const float mass = 1; // kg rocket mass
const float area_normal = 1;// m^2, bit more that bt-80
const float Cd_normal = 1;// drag coefficient


// Constants for braking conditions
const float area_brake = 1;// m^2, adjusted area with brakes deployed
const float Cd_brake =  1;// drag coefficient with brakes deployed


Adafruit_BME280 bme;
Adafruit_MPU6050 mpu;
Servo brakeServo;


float velocityY = 0;
unsigned long lastTime = 0;
bool brakesDeployed = false;


const int SERVO_PIN = PIN;  // Replace with actual pin
const int SERVO_CLOSED_ANGLE = CLOSED_ANGLE;// Update this should be 0?
const int SERVO_DEPLOYED_ANGLE = DEPLOYED_ANGLE;// Update this


void setup() {


 Serial.begin(9600);
 Wire.begin();


 if (!bme.begin(0x76)) {
   Serial.println("BME280 not found!");
   while (1);
 }


 mpu.begin();


 brakeServo.attach(SERVO_PIN);
 brakeServo.write(SERVO_CLOSED_ANGLE);


 lastTime = millis();


 Serial.println("Time(ms),Height(m),AccelY(g),VelocityY(m/s),PredictedApogee(m),BrakesDeployed");
}


// Calculate drag acceleration for given velocity and drag coefficient
 float dragAccel(float velocity, float Cd, float area) {
 float dragForce = 0.5 * AIR_DENSITY * Cd * area * velocity * velocity;
 float dragAcc = dragForce / mass;
 // Drag always opposes velocity direction
 if (velocity > 0)
   return -dragAcc;
 else
   return dragAcc;  // if falling down, drag pushes upward
}


// Predict apogee by simulating motion until velocity reaches zero or below
 float predictApogee(float currentHeight, float currentVelocity, bool brakesNow, float area, float Cd) {
 float simHeight = currentHeight;
 float simVelocity = currentVelocity;
 float dt = 0.01;  // simulation time step (s)
 while (simVelocity > 0) {
   float a_gravity = -9.81;
   float a_drag = dragAccel(simVelocity, Cd, area);
   float a_total = a_gravity + a_drag;


   simVelocity += a_total * dt;
   simHeight += simVelocity * dt;
 if(simHeight < 0)
   break;  // safety in case of weird sim
}
return simHeight;


}


void loop() {
 unsigned long currentTime = millis();
 float dt = (currentTime - lastTime) / 1000.0;
 lastTime = currentTime;


 // Read altitude
 float pressure = bme.readPressure();
 float height = 44330.0 * (1.0 - pow(pressure / (SEALEVEL_PRESSURE_HPA * 100.0), 0.1903));


 sensors_event_t a, g, temp;
 mpu.getEvent(&a, &g, &temp);




 float accelY_g = a.acceleration.y / 16384.0;
 float accelY_mss = accelY_g * 9.80665;
 float netAccelY = accelY_mss - 9.81; // Remove gravity


 // Determine current Cd and area based on brakes deployment
 float currentCd = brakesDeployed ? Cd_brake : Cd_normal;
 float currentArea = brakesDeployed ? area_brake : area_normal;


 // Calculate drag acceleration with current Cd and area
 float dragAcc = dragAccel(velocityY, currentCd, currentArea);


 velocityY += netAccelY * dt;


 // Predict apogee if brakes deploy now
 float predictedApogeeIfBrakeNow = predictApogee(height, velocityY, true, currentArea, Cd_brake);


 // Predict apogee if brakes never deployed
 float predictedApogeeNoBrake = predictApogee(height, velocityY, false, area_normal, Cd_normal);


 // Deploy brakes if latest point that predicted apogee ≈ target
 if (!brakesDeployed && predictedApogeeIfBrakeNow <= TARGET_APOGEE_M) {
   brakeServo.write(SERVO_DEPLOYED_ANGLE);
   brakesDeployed = true;
 }


 // Log data
 Serial.print(currentTime);
 Serial.print(",");
 Serial.print(height);
 Serial.print(",");
 Serial.print(accelY_g, 4);
 Serial.print(",");
 Serial.print(velocityY, 4);
 Serial.print(",");
 Serial.print(predictedApogeeIfBrakeNow, 4);
 Serial.print(",");
 Serial.println(brakesDeployed ? 1 : 0);
 delay(50);
}
