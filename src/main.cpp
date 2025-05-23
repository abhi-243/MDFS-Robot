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

#define IN1 30
#define IN2 26
#define IN3 28
#define IN4 24

#define FFlapServoPin 5   // Flap Servo Pin
#define BFlapServoPin 6   // Flap Servo Pin
#define FArmServoPin 10      // Arm Servo Pin
#define BArmServoPin 3       // Arm Servo Pin

//max vel and max accel
#define maxSpeedArms 500
#define accelArms 1750

#define motorInterfaceType 1 // Using driver with step/dir interface

// ===================== Stepper Motor Instances =====================
AccelStepper FRstepper(motorInterfaceType, FRStep, FRDir);  // Front Right
AccelStepper FLstepper(motorInterfaceType, FLStep, FLDir);  // Front Left
AccelStepper BRstepper(motorInterfaceType, BRStep, BRDir);  // Back Right
AccelStepper BLstepper(motorInterfaceType, BLStep, BLDir);  // Back Left
AccelStepper FArmStepper(motorInterfaceType, FArmStep, FArmDir); // Front Arm (unused here)
AccelStepper BArmStepper(motorInterfaceType, BArmStep, BArmDir); // Back Arm (unused here)
AccelStepper CrslStepper(AccelStepper::FULL4WIRE, IN1,IN2,IN3,IN4); // Stepper for the carousel 

// ===================== Servo Instances =======================
Servo ejectorServo;     // Ejector Servo
Servo FrontFlapServo;   // Front Flap Servo
Servo BackFlapServo;    // Back Flap Servo
Servo FrontArmServo;    // Front Arm Servo
Servo BackArmServo;     // Back Arm Servo

// ===================== Motion Parameters =====================
const float wheelDiameterMM = 80.0;                 // Wheel diameter in mm
const float stepsPerRevolution = 200 * 4;           // 200 steps/rev * 4 (microstepping = 1/4 step)
const float wheelCircumference = PI * wheelDiameterMM; // Circumference = π * diameter
const float stepsPerMM = stepsPerRevolution / wheelCircumference; // Steps per mm of travel

int maxSpeed = 200; // Maximum speed for steppers
int maxAccel = 5000;    // Acceleration for steppers

int armAngle = 0;
bool armFloppers = false;

// ===================== Timing Variables =======================
unsigned long nowTime = 0;

unsigned long gateTimePrev = 0; // for Aidan's servo gate locks
const int lockDelay = 5;

unsigned long carouselRotPrev = 0;
unsigned long pistonLast = 0;
const int pistonDelay = 1000;
const int carouselDelay = 5000;

unsigned long armTimePrev = 0;
const int armDelay = 2000;

// ===================== General Variables ======================
int carouselCycles = 0;
bool carouselEject = false;
int pistonPosition = 180;
int carouselState = 0; //0 = move, 1 = extend, 2 = retract;

bool moving = true;

// ===================== Direction Settings =====================
// Use 1 or -1 to reverse motor direction to match real-world wiring
int FrontRight = -1;
int FrontLeft  = -1;
int BackRight  = 1;
int BackLeft   = -1;
int FArm = 1;
int BArm = 1;

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
  FRstepper.setMaxSpeed(maxSpeed); FRstepper.setAcceleration(maxAccel);
  FLstepper.setMaxSpeed(maxSpeed); FLstepper.setAcceleration(maxAccel);
  BRstepper.setMaxSpeed(maxSpeed); BRstepper.setAcceleration(maxAccel);
  BLstepper.setMaxSpeed(maxSpeed); BLstepper.setAcceleration(maxAccel);
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

// ================================= Carousel control Code ==========================

void carouselHandler()
{
  CrslStepper.runSpeedToPosition();
  CrslStepper.setSpeed(300);

  switch(carouselState)
  {
    case 0:
    if(carouselCycles > 0 && (nowTime - carouselRotPrev) >= carouselDelay)
    {
      CrslStepper.move(683);
      carouselCycles = carouselCycles - 1;
      carouselRotPrev = nowTime;
      if(carouselEject == true)
      {
        carouselState = 1;
        pistonLast = nowTime;
      }
    }
    break;

    case 1: //Pre piston delay based on rotate speed and steps required
    if(nowTime - pistonLast >= 2780)
    {
      pistonLast = nowTime;
      carouselState = 2;
    }
    break;

    case 2:
    if(nowTime - pistonLast > pistonDelay)
    {
      pistonPosition = 100;
      pistonLast = nowTime;

      carouselState = 3;
    }
    break;

    case 3:
    if(nowTime - pistonLast > pistonDelay)
    {
      pistonPosition = 180;
      carouselRotPrev = nowTime;
      carouselState = 0;
    }
    break;
  }
}

// =================== Angle function ====================
long angleToSteps(float angle) {
  return round((angle / 360.0) * stepsPerRevolution);
}

// =================== Move Arms ======================
void moveArmSystem(float angleDegrees) {
  long frontSteps = angleToSteps(angleDegrees);
  long backSteps = -frontSteps;

  // Move steppers
  FArmStepper.move(frontSteps);
  BArmStepper.move(backSteps);

  // Set servos: rotate opposite direction
  if (angleDegrees > 0) {
    // Front going down, back going up
    FrontArmServo.write(60);  // rotate CCW
    BackArmServo.write(120);  // rotate CW
  } else {
    FrontArmServo.write(120); // rotate CW
    BackArmServo.write(60);   // rotate CCW
  }

  // While steppers are moving
  while (FArmStepper.isRunning() || BArmStepper.isRunning()) {
    FArmStepper.run();
    BArmStepper.run();
  }

  // Stop servos when steppers stop
  FrontArmServo.write(90);
  BackArmServo.write(90);
}

// ===================== Setup Function =====================
void setup() {
  Serial.begin(115200); // Start serial communication for debugging

  ejectorServo.attach(11);
  ejectorServo.write(180);

  FrontFlapServo.attach(FFlapServoPin);
  BackFlapServo.attach(BFlapServoPin);
  FrontArmServo.attach(FArmServoPin);
  BackArmServo.attach(BArmServoPin);

  // Set all enable pins to OUTPUT mode
  pinMode(FREn, OUTPUT); pinMode(FLEn, OUTPUT);
  pinMode(BREn, OUTPUT); pinMode(BLEn, OUTPUT);
  pinMode(FArmEn, OUTPUT); pinMode(BArmEn, OUTPUT);

  // Enable all motors (LOW signal enables stepper drivers)
  digitalWrite(FREn, LOW); digitalWrite(FLEn, LOW);
  digitalWrite(BREn, LOW); digitalWrite(BLEn, LOW);
  digitalWrite(FArmEn, LOW); digitalWrite(BArmEn, LOW);

  // Configure speed and acceleration for each motor
  FRstepper.setMaxSpeed(maxSpeed); FRstepper.setAcceleration(maxAccel);
  FLstepper.setMaxSpeed(maxSpeed); FLstepper.setAcceleration(maxAccel);
  BRstepper.setMaxSpeed(maxSpeed); BRstepper.setAcceleration(maxAccel);
  BLstepper.setMaxSpeed(maxSpeed); BLstepper.setAcceleration(maxAccel);
  FArmStepper.setMaxSpeed(maxSpeedArms); 
  FArmStepper.setAcceleration(accelArms);
  BArmStepper.setMaxSpeed(maxSpeedArms); 
  BArmStepper.setAcceleration(accelArms);
  CrslStepper.setMaxSpeed(600);
  CrslStepper.setSpeed(100);

  //-------------------- CAROUSEL TESTING REMOVE LATER -------------------
  carouselCycles = 3;
  carouselEject = true;
}

// ===================== Main Loop =====================
void loop() {
  runAllSteppers(); // Continuously run all motors
  carouselHandler();
  ejectorServo.write(pistonPosition);

  nowTime = millis(); // Current time in ms

  if (nowTime - armTimePrev >= armDelay)
  {
    armTimePrev = nowTime;
    if (moving)
    {
      moveArmSystem(45);
    }
    else
    {
      moveArmSystem(-45);
    }
  }

  //------------------------------------------ Sweep Code to test Aidans gate servos
  /*
  if(nowTime - gateTimePrev >= lockDelay)
  {
    if(armFloppers == false)
    {
      armAngle = armAngle + 1;
      if(armAngle >= 165)
      {
        armFloppers = true;
      }
    }
    else{
      armAngle = armAngle -1;
      if(armAngle <= 0)
      {
        armFloppers = false;
      }
    }
    gateTimePrev = nowTime;
  }

  BackFlapServo.write(armAngle);
  FrontFlapServo.write(165 - armAngle);
  */
  //-------------------------------------------End Sweep test

  delayMicroseconds(100); // Brief pause to avoid CPU overload
}