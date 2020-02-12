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

}

void GEAH::Motor::runPid() {
  double dt = time(0) - prevTime;
  prevTime = time(0);
  double accel = (get_actual_velocity() - prevVelocity)/dt;
  double y = get_position()-pidTarget;

  double porportion = (*speedModifier) * kP * y;
  double derivative =  ( -kD * accel / get_position());
  double integral = ( kI * ((porportion + derivative) - accel) );

  double PID = porportion + integral + derivative;

  move_voltage(callibrationSettings::kM * PID);

  std::cout << "dt: " << dt << " time: " << prevTime << " accel: " << accel << " y: " << y << " porportion: " << porportion << " integral: " << integral << " derivative: " << derivative << " PID: " << PID << " moveVoltage: " << callibrationSettings::kM * PID;
  prevVelocity = get_actual_velocity();
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
