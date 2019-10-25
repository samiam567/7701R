#ifndef GENERAL_FUNCTIONS_H
#define GENERAL_FUNCTIONS_H
#include "main.h"

int getAuton(); //returns what the auton-pot indicates which auton will run and is in autonomous.cpp

void LEDs(bool state); //controls the LEDs (in initalize.cpp)

void moveMotor(pros::Motor motor, float magnatude, int speed, int type);

//void setMotorPosition(pros::Motor motor, float position, int speed, int type);

void autonomous();

void checkForStop();

int getDrive();

int getAuton(); //(in initialize.cpp)

struct driver {
  std::string name;
  int drive_type; //0 = arcade, 1 = tank, 2 = drone
  pros::controller_digital_e_t CANNON_UP, CANNON_DOWN;
  pros::controller_digital_e_t FIRE_CANNON;
  pros::controller_digital_e_t HIGH_FLIP_UP,HIGH_FLIP_DOWN;
  pros::controller_digital_e_t LAUNCH_AUTON;
  int LED_CONFIG;

};


driver getDriver(); //(in initialize.cpp)

#endif
