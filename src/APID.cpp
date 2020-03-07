#include "APID.hpp"
#include "main.h"
#include "generalFunctions.h"


GEAH::APID::APID() {
  kP = 1.9f;
  kI = 0.01f;
  kD = 0.4f;
  kM = 1.0f;

  input = &one;
  output = &one;
  porportion = 0;
  integral = 0;
  derivative = 0;
  prevVelocity = 0;
  pidTarget = one;
  prevY = 0;
  prevTime = 0;
  speedModifier = &one; //set speedModifier to SOMETHING. This should be changed later
  counter = 0;
}

GEAH::APID::APID(double pidTarget, double* input, double* output, double kP, double kI, double kD) {
   APID::pidTarget = pidTarget;
   APID::input = input;
   APID::output = output;
   APID::kP = kP;
   APID::kI = kI;
   APID::kD = kD;
   APID::kM = 1;

   porportion = 0;
   integral = 0;
   derivative = 0;
   prevVelocity = 0;
   prevY = 0;
   prevTime = 0;
   speedModifier = &one; //set speedModifier to SOMETHING. This should be changed later
   counter = 0;


}

double GEAH::APID::runPid(double dt) {

  bool pidTuningInProgress = false;
  // HOW TO TUNE:
  //Adjust kM in settings until acccel = PID
  //increase kI if oscilations are lopsided
  //increase kD if system is oscillating
  //increase kP if the system doesn't reach target position fast enough


  double y = pidTarget-(*input);

  porportion = std::abs(*speedModifier) * kP * y;
  derivative = kD * (y-prevY)/dt;
  integral += kI * ((y-prevY) * dt);

  double PID = (porportion + integral + derivative);


  (*output) = kM*PID; //but the autons are currently written for this one (3/4/2020)

   if (pidTuningInProgress) {
     double velocity = (y-prevY)/dt;
     double accel = (velocity - prevVelocity)/dt;
     std::cout << "actualV: " << velocity << " pos: " << (*input) << std::endl;
     std::cout << " counter: " << counter << " dt: " << dt << " time: " << prevTime << " accel: " << accel << " y: " << y << " porportion: " << porportion << " integral: " << integral << " derivative: " << derivative << " PID: " << PID << " moveVoltage: " << kM * PID << std::endl;
      prevVelocity = velocity;
   }

   return *output;
}

void GEAH::APID::resetPID() {
  integral = 0;
}
void GEAH::APID::setAPIDConstants(double nkP, double nkI, double nkD) {
  GEAH::APID::kP = nkP;
  GEAH::APID::kI = nkI;
  GEAH::APID::kD = nkD;
}

void GEAH::APID::setAPIDTarget(double targ) {
  pidTarget = targ;
}

void GEAH::APID::setInputPointer(double *inputValue) {
  input = inputValue;
}

void GEAH::APID::setOutputPointer(double *outputPointer) {
  output = outputPointer;
}
double* GEAH::APID::getOutputPointer() {
  return output;
}
void GEAH::APID::setSpeedModifier(double *newSpeedModifier) {
  speedModifier = newSpeedModifier;
}

void GEAH::APID::setKM(float kM) {
  GEAH::APID::kM = kM;
}
