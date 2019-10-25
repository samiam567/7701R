#ifndef SETTINGS_H
#define SETTINGS_H

namespace ports{
  //drivetrain
  constexpr int LEFT_WHEEL_BACK_PORT = 1;
  constexpr int RIGHT_WHEEL_BACK_PORT = 2;
  constexpr int LEFT_WHEEL_FRONT_PORT = 3;
  constexpr int RIGHT_WHEEL_FRONT_PORT = 4;
    //note: for two wheel drive, you should be able to set the front and back motor ports for each side equal to the same port

  //other
  constexpr int LEFT_ARM_PORT = 3;
  constexpr int RIGHT_ARM_PORT = 4;
  constexpr int CLAW_PORT = 5;
  constexpr int CANNON_AIM_PORT = 6;
  constexpr int BALL_FIRE_PORT = 6;
  constexpr int HIGH_FLIPPER_PORT = 8;
  constexpr int AUTON_POTENTIOMETER_PORT = 2;
  constexpr int DRIVE_POTENTIOMETER_PORT = 8;

  //LEDs
  constexpr int LED_1 = 1;
  constexpr int LED_2 = 2;
  constexpr int LED_3 = 3;
  constexpr int LED_4 = 5;

  //ultrasonic
  constexpr int ULTRASONIC_ECHO_PORT = 8;
  constexpr int ULTRASONIC_PING_PORT = 7;


}

namespace callibrationSettings {
    constexpr double TURN_CORRECTION = 0.7;
    constexpr double MOTOR_POSITION_ERROR = 3; //deviation in degrees from where the motor is set to be
}
#endif
