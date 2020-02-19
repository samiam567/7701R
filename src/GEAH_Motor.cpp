#include "GEAH_Motor.hpp"
#include "Settings.h"
#include <ctime>
#include <cmath>
#include "generalFunctions.h"

GEAH::Motor::Motor(std::string Name, const std::uint8_t port, const pros::motor_gearset_e_t gearset, const std::uint8_t isReversed, const pros::motor_encoder_units_e_t encoderMode) : pros::Motor(port,gearset,isReversed,encoderMode) {
  name = Name;
  std::cout << name << " has been created \n";
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
