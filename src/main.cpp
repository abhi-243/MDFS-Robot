#include <Arduino.h>
#include <AccelStepper.h>

const int dirPin1 = 33;
const int stepPin1 = 32;
const int dirPin2 = 35;
const int stepPin2 = 34;


AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2);

enum State
{
  MOVE_TO_FIRST,
  WAIT_AFTER_FIRST,
  MOVE_TO_SECOND,
  WAIT_AFTER_SECOND,
  RETURN_HOME
};

State currentState = MOVE_TO_FIRST;
unsigned long waitStartTime = 0;

void setup()
{
  stepper1.setMaxSpeed(400);
  stepper1.setAcceleration(50);
  stepper2.setMaxSpeed(400);
  stepper2.setAcceleration(50);

  Serial.begin(9600);

  stepper1.setCurrentPosition(0);  // Set starting position to 0
  stepper2.setCurrentPosition(0);  // Set starting position to 0
}


void loop()
{
  stepper1.run();
  stepper2.run();

  switch (currentState) {
    case MOVE_TO_FIRST:
      if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() ==0) {
        waitStartTime = millis();
        currentState = WAIT_AFTER_FIRST;
      } else {
        stepper1.moveTo(500);
        stepper2.moveTo(500);
      }
      break;

    case WAIT_AFTER_FIRST:
      if (millis() - waitStartTime >= 2000) {  // wait 2 seconds
        currentState = MOVE_TO_SECOND;
      }
      break;

    case MOVE_TO_SECOND:
      if (stepper1.distanceToGo() == 0  && stepper2.distanceToGo() == 0) {
        waitStartTime = millis();
        currentState = WAIT_AFTER_SECOND;
      } else {
        stepper1.moveTo(1000); // Move further to position 1000
        stepper2.moveTo(100);
      }
      break;

    case WAIT_AFTER_SECOND:
      if (millis() - waitStartTime >= 2000) {  // wait 2 seconds
        currentState = RETURN_HOME;
      }
      break;

    case RETURN_HOME:
      if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) {
        // Optionally restart or stop
        // currentState = MOVE_TO_FIRST;  // If you want it to repeat
      } else {
        stepper1.moveTo(0);
        stepper2.moveTo(0);
      }
      break;
  }
}