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

  porportion = 0;
  integral = 0;
  derivative = 0;
  prevVelocity = 0;
  pidTarget = 0;
  prevTime = time(0);
  speedModifier = &one; //set speedModifier to SOMETHING. This should be changed later
  counter = 0;
}

void GEAH::Motor::runPid() {

  bool pidTuningInProgress = true;
  // HOW TO TUNE:
  //Adjust kM in settings until acccel = PID
  //increase kI if oscilations are lopsided
  //increase kD if system is oscillating
  //increase kP if the system doesn't reach target position fast enough

  double dt = 5;//time(0) - prevTime;
  prevTime = time(0);
  double velocity = get_actual_velocity() * 360 / 60;
  double accel = (get_actual_velocity() - prevVelocity)/dt;
  double y = pidTarget-get_position();

  porportion = (*speedModifier) * kP * y;
  derivative = -(kD * velocity / y);

  integral = 0;
  //integral = integral*counter + ( kI * ((porportion + derivative) - accel)*dt );
  //integral /= counter++;

  double PID = porportion + integral + derivative;

  move(callibrationSettings::kM * PID);

   std::cout << "actualV: " << velocity << " pos: " << get_position() << std::endl;
   if (pidTuningInProgress) std::cout << " counter: " << counter << " dt: " << dt << " time: " << prevTime << " accel: " << accel << " y: " << y << " porportion: " << porportion << " integral: " << integral << " derivative: " << derivative << " PID: " << PID << " moveVoltage: " << callibrationSettings::kM * PID << std::endl;

   prevVelocity = velocity;


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
