#include "GEAH_Motor.hpp"
#include "Settings.h"
#include <ctime>
#include <cmath>
#include "generalFunctions.h"

GEAH::Motor::Motor(std::string Name, const std::uint8_t port, const pros::motor_gearset_e_t gearset, const std::uint8_t isReversed, const pros::motor_encoder_units_e_t encoderMode) : pros::Motor(port,gearset,isReversed,encoderMode) {
  name = Name;
  std::cout << name << " has been created \n";

  kP = 1.0f;
  kI = 0.0f;
  kD = 0.01f;
  kM = 1.0f;

  porportion = 0;
  integral = 0;
  derivative = 0;
  prevVelocity = 0;
  pidTarget = 0;
  prevY = 0;
  prevTime = time(0);
  speedModifier = &one; //set speedModifier to SOMETHING. This should be changed later
  counter = 0;
}

void GEAH::Motor::runPid() {

  bool pidTuningInProgress = false;
  // HOW TO TUNE:
  //Adjust kM in settings until acccel = PID
  //increase kI if oscilations are lopsided
  //increase kD if system is oscillating
  //increase kP if the system doesn't reach target position fast enough

  double dt = 2;//time(0) - prevTime;
  //prevTime = time(0);

  double y = pidTarget-get_position();

  porportion = (*speedModifier) * kP * y;
  integral += kI * ((y-prevY) * dt);
  derivative = kD * (y-prevY)/dt;


  double PID = (porportion + integral + derivative);


  move_velocity(kM * PID);

   if (pidTuningInProgress) {
     double velocity = get_actual_velocity() * 360 / 60;
     double accel = (velocity - prevVelocity)/dt;
     std::cout << "actualV: " << velocity << " pos: " << get_position() << std::endl;
     std::cout << " counter: " << counter << " dt: " << dt << " time: " << prevTime << " accel: " << accel << " y: " << y << " porportion: " << porportion << " integral: " << integral << " derivative: " << derivative << " PID: " << PID << " moveVoltage: " << kM * PID << std::endl;
     prevVelocity = velocity;
   }


}

void GEAH::Motor::resetPID() {
  integral = 0;
}
void GEAH::Motor::setAPIDConstants(double nkP, double nkI, double nkD) {
  GEAH::Motor::kP = nkP;
  GEAH::Motor::kI = nkI;
  GEAH::Motor::kD = nkD;
}

void GEAH::Motor::setAPIDTarget(double targ) {
  pidTarget = targ;
}

void GEAH::Motor::setSpeedModifier(double *newSpeedModifier) {
  speedModifier = newSpeedModifier;
}

void GEAH::Motor::setKM(float kM) {
  GEAH::Motor::kM = kM;
}

void GEAH::Motor::operator= (int voltage ) {
  GEAH::Motor::move(voltage);
}
void GEAH::Motor::operator= (double voltage ) {
  GEAH::Motor::move(voltage);
}
void GEAH::Motor::operator= (long voltage ) {
  GEAH::Motor::move(voltage);
}

void GEAH::Motor::setPreCallibrationPos(double preCalPos) {
  GEAH::Motor::preCallibrationPos = preCalPos;
}

double GEAH::Motor::getPreCallibrationPos() {
  return preCallibrationPos;
}

std::string GEAH::Motor::getName() {
  return name;
}
