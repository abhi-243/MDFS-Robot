#include <Arduino.h>
#include <AccelStepper.h>

const int dirPin = 2;
const int stepPin = 15;

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

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
  stepper.setMaxSpeed(400);
  stepper.setAcceleration(50);

  Serial.begin(9600);

  stepper.setCurrentPosition(0);  // Set starting position to 0
}


void loop()
{
  stepper.run();

  switch (currentState) {
    case MOVE_TO_FIRST:
      if (stepper.distanceToGo() == 0) {
        waitStartTime = millis();
        currentState = WAIT_AFTER_FIRST;
      } else {
        stepper.moveTo(500);
      }
      break;

    case WAIT_AFTER_FIRST:
      if (millis() - waitStartTime >= 2000) {  // wait 2 seconds
        currentState = MOVE_TO_SECOND;
      }
      break;

    case MOVE_TO_SECOND:
      if (stepper.distanceToGo() == 0) {
        waitStartTime = millis();
        currentState = WAIT_AFTER_SECOND;
      } else {
        stepper.moveTo(1000); // Move further to position 1000
      }
      break;

    case WAIT_AFTER_SECOND:
      if (millis() - waitStartTime >= 2000) {  // wait 2 seconds
        currentState = RETURN_HOME;
      }
      break;

    case RETURN_HOME:
      if (stepper.distanceToGo() == 0) {
        // Optionally restart or stop
        // currentState = MOVE_TO_FIRST;  // If you want it to repeat
      } else {
        stepper.moveTo(0);
      }
      break;
  }
}