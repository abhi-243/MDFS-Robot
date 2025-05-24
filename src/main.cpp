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
#define FArmServoPin 4      // Arm Servo Pin
#define BArmServoPin 3       // Arm Servo Pin

#define motorInterfaceType 1 // Using driver with step/dir interface

// ===================== Stepper Motor Instances =====================
AccelStepper FRstepper(motorInterfaceType, FRStep, FRDir);  // Front Right
AccelStepper FLstepper(motorInterfaceType, FLStep, FLDir);  // Front Left
AccelStepper BRstepper(motorInterfaceType, BRStep, BRDir);  // Back Right
AccelStepper BLstepper(motorInterfaceType, BLStep, BLDir);  // Back Left
AccelStepper FArmStepper(motorInterfaceType, FArmStep, FArmDir); // Front Arm (unused here)
AccelStepper BArmStepper(motorInterfaceType, BArmStep, BArmDir); // Back Arm (unused here)
AccelStepper CrslStepper(AccelStepper::FULL4WIRE, IN1,IN2,IN3,IN4); // Stepper for the carousel 

// =================== Servo Smoother ====================
const float servoRPM = 4;
const float servoDelay = 60000.0 / (servoRPM * 180.0); // in milliseconds
struct SmoothServoMover {
  Servo* servo;
  int currentAngle;
  int targetAngle;
  unsigned long lastMoveTime = 0;
  bool active = false;
  
  void attach(Servo& s, int pin, int initialAngle) {
    servo = &s;
    currentAngle = initialAngle;
    targetAngle = initialAngle;
    servo->attach(pin);
    servo->write(initialAngle);
    lastMoveTime = millis(); // Initialize timing properly
    active = false;
  }
  
   void moveTo(int angle) {
    targetAngle = angle;
    active = true;
    lastMoveTime = millis();
  }

  void update(unsigned long nowTime) {
    if (!active) return;

    if (nowTime - lastMoveTime >= servoDelay) {
      lastMoveTime = nowTime;

      if (currentAngle < targetAngle) currentAngle++;
      else if (currentAngle > targetAngle) currentAngle--;

      servo->write(currentAngle);

      if (currentAngle == targetAngle) {
        active = false;
      }
    }
  }

    bool isMoving() {
    return active;
  }
};

// ===================== Servo Instances =======================
Servo ejectorServo;     // Ejector Servo
Servo FrontFlapServo;   // Front Flap Servo
Servo BackFlapServo;    // Back Flap Servo
Servo FrontArmServo;
Servo BackArmServo;
SmoothServoMover FrontServoMover;
SmoothServoMover BackServoMover;

// ===================== Motion Parameters =====================
const float wheelDiameterMM = 80.0;                 // Wheel diameter in mm
const float stepsPerRevolution = 200 * 4;           // 200 steps/rev * 4 (microstepping = 1/4 step)
const float wheelCircumference = PI * wheelDiameterMM; // Circumference = π * diameter
const float stepsPerMM = stepsPerRevolution / wheelCircumference; // Steps per mm of travel

int maxSpeed = 200; // Maximum speed for steppers
int maxArmSpeed = 45; //Maximum speed for arm steppers
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

int armState = 0;

bool hasMoved = false;

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
  FArmStepper.run();
  BArmStepper.run();
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

// =================== Angle Calc ========================
long angleToSteps(float angle) {
  return round((angle / 360.0) * stepsPerRevolution);
}

// =================== Move Arms ======================
void moveArmSystem(float angleDegrees) {
  long Steps = angleToSteps(angleDegrees);

  // Move steppers
  FArmStepper.move(Steps);
  BArmStepper.move(Steps);

  // Move servos smoothly in opposite directions
  if (angleDegrees > 0) {
    FrontServoMover.moveTo(170);
    BackServoMover.moveTo(10);
  } else {
    FrontServoMover.moveTo(10);
    BackServoMover.moveTo(170);
  }
}

// ===================== Setup Function =====================
void setup() {
  Serial.begin(115200); // Start serial communication for debugging

  ejectorServo.attach(11);
  ejectorServo.write(180);

  FrontFlapServo.attach(FFlapServoPin);
  BackFlapServo.attach(BFlapServoPin);

  FrontServoMover.attach(FrontArmServo, FArmServoPin, 0);
  BackServoMover.attach(BackArmServo, BArmServoPin, 170);

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
  FArmStepper.setMaxSpeed(maxArmSpeed); 
  FArmStepper.setAcceleration(maxAccel);
  FArmStepper.setCurrentPosition(0);
  BArmStepper.setMaxSpeed(maxArmSpeed); 
  BArmStepper.setCurrentPosition(0);
  BArmStepper.setAcceleration(maxAccel);
  CrslStepper.setMaxSpeed(600);
  CrslStepper.setSpeed(100);

  //-------------------- CAROUSEL TESTING REMOVE LATER -------------------
  carouselCycles = 3;
  carouselEject = true;
}

// ===================== Main Loop =====================
void loop() {
  nowTime = millis(); // Current time in ms
  runAllSteppers(); // Continuously run all motors
  carouselHandler();
  ejectorServo.write(pistonPosition);

  // Update smooth servos
  FrontServoMover.update(nowTime);
  BackServoMover.update(nowTime);

  switch (armState) {
    case 0:
      moveArmSystem(170);
      armState = 1;
      armTimePrev = nowTime;
      break;

    case 1:
      if ((nowTime - armTimePrev >= armDelay) &&
          !FArmStepper.isRunning() &&
          !BArmStepper.isRunning() &&
          !FrontServoMover.isMoving() &&
          !BackServoMover.isMoving()) {
        armState = 2;
        armTimePrev = nowTime;
      }
      break;

    case 2:
      moveArmSystem(-170);
      armState = 3;
      armTimePrev = nowTime;
      break;

    case 3:
      if ((nowTime - armTimePrev >= armDelay) &&
          !FArmStepper.isRunning() &&
          !BArmStepper.isRunning() &&
          !FrontServoMover.isMoving() &&
          !BackServoMover.isMoving()) {
        armState = 4;
      }
      break;

    case 4:
      // Stop forever
      break;
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