#include "Arduino.h"
#include <AccelStepper.h>
#include <Wire.h>
#include <MPU9250.h>

MPU9250 mpu;

const int MPU_ADDR = 0x68;

float yawOffset = 0;
float lastYaw = 0;
const float YAW_TOLERANCE = 1.5;  // degrees
const int BUTTON_PIN = 2;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  pinMode(BUTTON_PIN, INPUT); // setup pin 2 for input

  Wire.begin();
  Wire.setClock(400000);

  delay(1000);

  if (!mpu.setup(MPU_ADDR)) {
    Serial.println("MPU9250 not found. Halting.");
    while (1);
  }

  mpu.verbose(false);
  mpu.calibrateAccelGyro();
  mpu.calibrateMag();
}

void loop() {
  if (!mpu.update()) return;

  // Button pressed = tare yaw
  if (digitalRead(BUTTON_PIN) == HIGH) {
    yawOffset = mpu.getYaw();
  }

  float currentYaw = mpu.getYaw();
  float correctedYaw = currentYaw - yawOffset;

  // Keep yaw within -180 to +180
  if (correctedYaw > 180) correctedYaw -= 360;
  if (correctedYaw < -180) correctedYaw += 360;

  if (abs(correctedYaw - lastYaw) > YAW_TOLERANCE) {
    lastYaw = correctedYaw;
  }

  Serial.print(lastYaw, 2);
  Serial.print(",");
  Serial.print(mpu.getPitch(), 2);
  Serial.print(",");
  Serial.println(mpu.getRoll(), 2);

  delay(10);
}


//#define FRDir 38
//#define FRStep 40
//#define FLDir 45
//#define FLStep 47
//#define BRDir 51
//#define BRStep 53
//#define BLDir 39
//#define BLStep 41
//#define FArmDir 50
//#define FArmStep 52
//#define BArmDir 44
//#define BArmStep 46
//
//#define FREn 36  // Front Right
//#define FLEn 43  // Front Left
//#define BREn 49  // Back Right
//#define BLEn 37  // Back Left
//#define FArmEn 48 //Front Arm
//#define BArmEn 42 // Back Arm
//
//#define motorInterfaceType 1 //Driver with step and direction pins
//
//AccelStepper stepper1(motorInterfaceType, FRStep, FRDir); // Front Right
//AccelStepper stepper2(motorInterfaceType, FLStep, FLDir); // Front Left
//AccelStepper stepper3(motorInterfaceType, BRStep, BRDir); // Back Right
//AccelStepper stepper4(motorInterfaceType, BLStep, BLDir); // Back Left
//AccelStepper stepper5(motorInterfaceType, FArmStep, FArmDir); // Back Right
//AccelStepper stepper6(motorInterfaceType, BArmStep, BArmDir); // Back Left
//
//int maxSpeed = 2000; // steps per second
//int midSpeed = 1500; // steps per second
//
//// Manually set direction: 1 for forward, -1 for reverse
//int FrontRight = -1;
//int FrontLeft  = -1;
//int BackRight  = -1;
//int BackLeft   = -1;
//int FrontArm  = -1;
//int BackArm   = 1;
//
//unsigned long startTime;
//enum Phase { PHASE1, PHASE2, PHASE3, DONE };
//Phase currentPhase = PHASE1;
//
//void setup() {
//  // Set enable pins as outputs
//  pinMode(FREn, OUTPUT);
//  pinMode(FLEn, OUTPUT);
//  pinMode(BREn, OUTPUT);
//  pinMode(BLEn, OUTPUT);
//  pinMode(FArmEn, OUTPUT);
//  pinMode(BArmEn, OUTPUT);
//
//  // Set motor directions and speeds
//  stepper1.setMaxSpeed(maxSpeed);
//  stepper1.setSpeed(FrontRight * maxSpeed);
//
//  stepper2.setMaxSpeed(maxSpeed);
//  stepper2.setSpeed(FrontLeft * maxSpeed);
//
//  stepper3.setMaxSpeed(maxSpeed);
//  stepper3.setSpeed(BackRight * maxSpeed);
//
//  stepper4.setMaxSpeed(maxSpeed);
//  stepper4.setSpeed(BackLeft * maxSpeed);
//
//  stepper5.setMaxSpeed(maxSpeed);
//  stepper5.setSpeed(FrontArm * maxSpeed);
//
//  stepper6.setMaxSpeed(maxSpeed);
//  stepper6.setSpeed(BackArm * maxSpeed);
//
//  // Enable all motors initially
//  digitalWrite(FREn, LOW); // Front Right
//  digitalWrite(FLEn, LOW); // Front Left
//  digitalWrite(BREn, LOW); // Back Right
//  digitalWrite(BLEn, LOW); // Back Left
//  digitalWrite(FArmEn, LOW); // Back Right
//  digitalWrite(BArmEn, LOW); // Back Left
//
//  startTime = millis();
//}
//
//void loop() {
//  unsigned long elapsed = millis() - startTime;
//
//  switch (currentPhase) {
//    case PHASE1:
//      // Run all motors for 3 seconds
//      stepper1.runSpeed();
//      stepper2.runSpeed();
//      stepper3.runSpeed();
//      stepper4.runSpeed();
//      stepper5.runSpeed();
//      stepper6.runSpeed();
//
//      if (elapsed >= 3000) {
//        // Disable back motors
//        digitalWrite(FREn, HIGH); // Back Right
//        digitalWrite(FLEn, HIGH); // Back Left
//        //reset speed
//        stepper3.setMaxSpeed(midSpeed);
//        stepper3.setSpeed(BackRight * midSpeed);
//        currentPhase = PHASE2;
//        startTime = millis();
//      }
//      break;
//
//    case PHASE2:
//      // Run front motors for 2 seconds
//      stepper3.runSpeed(); // Front Right
//      stepper4.runSpeed(); // Front Left
//
//      if (elapsed >= 5000) {
//        // Re-enable back motors
//                //reset speed
//        stepper3.setMaxSpeed(maxSpeed);
//        stepper3.setSpeed(BackRight * maxSpeed);
//        digitalWrite(BREn, LOW); // Back Right
//        currentPhase = PHASE3;
//        startTime = millis();
//      }
//      break;
//
//    case PHASE3:
//      // Run all motors for 10 seconds
//      stepper1.runSpeed();
//      stepper2.runSpeed();
//      stepper3.runSpeed();
//      stepper4.runSpeed();
//      stepper5.runSpeed();
//      stepper6.runSpeed();
//      
//      if (elapsed >= 1000) {
//        // Optionally stop everything after final phase
//        digitalWrite(FREn, HIGH);
//        digitalWrite(FLEn, HIGH);
//        digitalWrite(BREn, HIGH);
//        digitalWrite(BLEn, HIGH);
//        digitalWrite(FArmEn, HIGH);
//        digitalWrite(BArmEn, HIGH);
//        currentPhase = DONE;
//      }
//      break;
//
//    case DONE:
//      // All motors disabled — do nothing
//      break;
//  }
//}