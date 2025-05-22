#include <Arduino.h>
#include <AccelStepper.h>
#include <Servo.h>

// ===================== Motor Pin Definitions =====================
#define FRStep 40    // Front Right Step pin
#define FRDir 38     // Front Right Direction pin
#define FREn 36      // Front Right Enable pin

#define FLStep 47    // Front Left Step pin
#define FLDir 45     // Front Left Direction pin
#define FLEn 43      // Front Left Enable pin

#define BRStep 53    // Back Right Step pin
#define BRDir 51     // Back Right Direction pin
#define BREn 49      // Back Right Enable pin

#define BLStep 41    // Back Left Step pin
#define BLDir 39     // Back Left Direction pin
#define BLEn 37      // Back Left Enable pin

#define FArmStep 52  // Front Arm Step pin
#define FArmDir 50   // Front Arm Direction pin
#define FArmEn 48    // Front Arm Enable pin

#define BArmStep 46  // Back Arm Step pin
#define BArmDir 44   // Back Arm Direction pin
#define BArmEn 42    // Back Arm Enable pin

#define servoPin1 9
#define servoPin2 4
#define servoPin3 10
#define servoPin4 3

//max vel and accel
#define maxSpeedArms 500
#define accelArms 1750
 
//direction settings
#define FArm 1 //placeholder
#define BArm 1 //placeholder

#define motorInterfaceType 1 // Using driver with step/dir interface

// ===================== Stepper Motor Instances =====================
AccelStepper FRstepper(motorInterfaceType, FRStep, FRDir);  // Front Right
AccelStepper FLstepper(motorInterfaceType, FLStep, FLDir);  // Front Left
AccelStepper BRstepper(motorInterfaceType, BRStep, BRDir);  // Back Right
AccelStepper BLstepper(motorInterfaceType, BLStep, BLDir);  // Back Left
AccelStepper FArmStepper(motorInterfaceType, FArmStep, FArmDir); // Front Arm (unused here)
AccelStepper BArmStepper(motorInterfaceType, BArmStep, BArmDir); // Back Arm (unused here)
AccelStepper CrslStepper(AccelStepper::FULL4WIRE, 30,26,28,24); // Stepper for the carousel 

// ===================== Servo Instances =======================
Servo ejectorServo;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

// ===================== Motion Parameters =====================
const float wheelDiameterMM = 80.0;                 // Wheel diameter in mm
const float stepsPerRevolution = 200 * 4;           // 200 steps/rev * 4 (microstepping = 1/4 step)
const float wheelCircumference = PI * wheelDiameterMM; // Circumference = π * diameter
const float stepsPerMM = stepsPerRevolution / wheelCircumference; // Steps per mm of travel

int maxSpeed = 2000; // Maximum speed for steppers
int accel = 3500;    // Acceleration for steppers

// ===================== Timing Variables =======================
unsigned long nowTime = 0;

unsigned long gateTimePrev = 0; // for Aidan's servo gate locks
const int lockDelay = 50;

unsigned long carouselRotPrev = 0;
const int carouselDelay = 2000;

// ===================== General Variables ======================
int carouselCycles = 0;
bool carouselEject = false;

// ===================== Direction Settings =====================
// Use 1 or -1 to reverse motor direction to match real-world wiring
int FrontRight = -1;
int FrontLeft  = -1;
int BackRight  = 1;
int BackLeft   = -1;

// ===================== State Machine for Movements =====================
enum MovementState {
  MOVE_RIGHT, WAIT1,
  MOVE_LEFT, WAIT2,
  MOVE_FORWARD1, WAIT3,
  MOVE_FORWARD2, DONE
};

MovementState currentState = MOVE_RIGHT; // Starting state
unsigned long movementStartTime = 0;     // Time when last movement ended
bool movementStarted = false;            // Flag to indicate if current movement has been triggered

// ===================== Movement Function =====================
// Moves the robot a specified distance forward/backward and/or sideways (strafe)
void moveDistance(float forwardMM, float strafeMM) {
  float forwardSteps = forwardMM * stepsPerMM;
  float strafeSteps = strafeMM * stepsPerMM;

  // Mecanum wheel step calculation:
  // Each wheel contributes differently to the combined movement vector
  long flSteps = (long)(forwardSteps + strafeSteps);
  long frSteps = (long)(forwardSteps - strafeSteps);
  long blSteps = (long)(forwardSteps - strafeSteps);
  long brSteps = (long)(forwardSteps + strafeSteps);

  // Apply directional corrections based on wiring/orientation
  FRstepper.move(FrontRight * frSteps);
  FLstepper.move(FrontLeft * flSteps);
  BRstepper.move(BackRight * brSteps);
  BLstepper.move(BackLeft * blSteps);

  // Set speed and acceleration for each motor
  FRstepper.setMaxSpeed(maxSpeed); FRstepper.setAcceleration(accel);
  FLstepper.setMaxSpeed(maxSpeed); FLstepper.setAcceleration(accel);
  BRstepper.setMaxSpeed(maxSpeed); BRstepper.setAcceleration(accel);
  BLstepper.setMaxSpeed(maxSpeed); BLstepper.setAcceleration(accel);
}

// Runs all steppers each loop iteration to progress toward target positions
void runAllSteppers() {
  FRstepper.run();
  FLstepper.run();
  BRstepper.run();
  BLstepper.run();
}

// Check if any of the steppers are still running a motion
bool steppersAreRunning() {
  // Return true if any motor still has distance to cover
  return FRstepper.isRunning() || FLstepper.isRunning() ||
         BRstepper.isRunning() || BLstepper.isRunning();
}

void carouselHandler()
{
  CrslStepper.setSpeed(300);
  CrslStepper.runSpeedToPosition();

  int currAction = 0; // 0 == eject, 1 == rotate
  switch (currAction)
  {
  case 0:
    if(carouselEject == true)
    {
      if(carouselRotPrev - nowTime >= carouselDelay / 4)
      {
        ejectorServo.write(90);
      }
      if(carouselRotPrev - nowTime >= carouselDelay / 2)
      {
        ejectorServo.write(0);
        carouselRotPrev = nowTime;
        currAction = 1;
      }
    }
    else 
    {
      currAction = 1;
    }
    break;
  
  case 1:
    if(carouselCycles > 0 && carouselRotPrev - nowTime >= carouselDelay)
    {
      CrslStepper.move(683);
      carouselCycles -= carouselCycles;
      carouselRotPrev = nowTime;
      if(carouselEject == true)
      {
        currAction = 0;
      }
    }
    break;
  }
}

// ===================Back arm function ====================

void backArmPosition (float angleBackDegrees){
  // Clamp angle
  if (angleBackDegrees < 0) angleBackDegrees += 360;
  if (angleBackDegrees >= 360) angleBackDegrees -= 360;
 
  long targetStepsBackArm = (angleBackDegrees / 360.0) * stepsPerRevolution;
  BArmStepper.move(targetStepsBackArm);
 
}

//===================== Front arm function

void frontArmPosition (float angleFrontDegrees){
  // Clamp angle
  if (angleFrontDegrees < 0) angleFrontDegrees += 360;
  if (angleFrontDegrees >= 360) angleFrontDegrees -= 360;
 
  long targetStepsFrontArm = (angleFrontDegrees / 360.0) * stepsPerRevolution;
  FArmStepper.move(targetStepsFrontArm);
 
}

// ===================== Setup Function =====================
void setup() {
  Serial.begin(115200); // Start serial communication for debugging

  ejectorServo.attach(11);

  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
  servo4.attach(servoPin4);

  // Set all enable pins to OUTPUT mode
  pinMode(FREn, OUTPUT); pinMode(FLEn, OUTPUT);
  pinMode(BREn, OUTPUT); pinMode(BLEn, OUTPUT);
  pinMode(FArmEn, OUTPUT); pinMode(BArmEn, OUTPUT);

  // Enable all motors (LOW signal enables stepper drivers)
  digitalWrite(FREn, LOW); digitalWrite(FLEn, LOW);
  digitalWrite(BREn, LOW); digitalWrite(BLEn, LOW);
  digitalWrite(FArmEn, LOW); digitalWrite(BArmEn, LOW);

  // Configure speed and acceleration for each motor
  FRstepper.setMaxSpeed(maxSpeed); FRstepper.setAcceleration(accel);
  FLstepper.setMaxSpeed(maxSpeed); FLstepper.setAcceleration(accel);
  BRstepper.setMaxSpeed(maxSpeed); BRstepper.setAcceleration(accel);
  BLstepper.setMaxSpeed(maxSpeed); BLstepper.setAcceleration(accel);
  FArmStepper.setMaxSpeed(maxSpeedArms); 
  FArmStepper.setAcceleration(accelArms);
  BArmStepper.setMaxSpeed(maxSpeedArms); 
  BArmStepper.setAcceleration(accelArms);
  CrslStepper.setMaxSpeed(600);

  //-------------------- CAROUSEL TESTING REMOVE LATER -------------------
  carouselCycles = 3;
  carouselEject = true;
}

// ===================== Main Loop =====================
void loop() {
  runAllSteppers(); // Continuously run all motors
  carouselHandler();

  nowTime = millis(); // Current time in ms

  // State machine for sequencing robot movements
  switch (currentState) {
    case MOVE_RIGHT:
      if (!movementStarted) {
        moveDistance(0, -170); // Strafe right by 170 mm
        movementStarted = true;
      }
      if (!steppersAreRunning()) {
        currentState = WAIT1; // Go to wait state
        movementStartTime = nowTime;
        movementStarted = false;
      }
      break;

    case WAIT1:
      // Wait 500ms after previous move before continuing
      if (nowTime - movementStartTime >= 500) {
        currentState = MOVE_LEFT;
      }
      break;

    case MOVE_LEFT:
      if (!movementStarted) {
        moveDistance(0, -160); // Strafe left by 160 mm (typo: maybe should be positive?)
        movementStarted = true;
      }
      if (!steppersAreRunning()) {
        currentState = WAIT2;
        movementStartTime = nowTime;
        movementStarted = false;
      }
      break;

    case WAIT2:
      if (nowTime - movementStartTime >= 500) {
        currentState = MOVE_FORWARD1;
      }
      break;

    case MOVE_FORWARD1:
      if (!movementStarted) {
        moveDistance(900, 0); // Move forward by 900 mm
        movementStarted = true;
      }
      if (!steppersAreRunning()) {
        currentState = WAIT3;
        movementStartTime = nowTime;
        movementStarted = false;
      }
      break;

    case WAIT3:
      if (nowTime - movementStartTime >= 1000) {
        currentState = MOVE_FORWARD2;
      }
      break;

    case MOVE_FORWARD2:
      if (!movementStarted) {
        moveDistance(800, 0); // Move forward by another 800 mm
        movementStarted = true;
      }
      if (!steppersAreRunning()) {
        currentState = DONE;
        movementStarted = false;
      }
      break;

    case DONE:
      // All motion complete. Idle state.
      break;
  }

  //------------------------------------------ Sweep Code to test Aidans gate servos
  /*for(int angle = 90; angle >= 180; angle--)
  {
    if(nowTime - gateTimePrev >= lockDelay)
    {
      servo1.write(angle);
      servo2.write(180-angle);
      gateTimePrev = nowTime;
    }
  }

    gateTimePrev = nowTime;
    for(int angle = 90; angle <= 180; angle++)
    {
      if(nowTime-gateTimePrev >= lockDelay)
      {
        servo1.write(angle);
        servo2.write(180-angle);
        gateTimePrev = nowTime;
      }
    }*/
  //---------------------------------------------------End Sweep test

  delayMicroseconds(100); // Brief pause to avoid CPU overload
}