#ifndef SETTINGS_H
#define SETTINGS_H

static const double ERROR = -0.001;

static const double version = 3.46;
namespace ports{
  //drivetrain
  constexpr int LEFT_WHEEL_BACK_PORT = 7;
  constexpr int RIGHT_WHEEL_BACK_PORT = 2;
  constexpr int LEFT_WHEEL_FRONT_PORT = 3;
  constexpr int RIGHT_WHEEL_FRONT_PORT = 9;
    //note: for two wheel drive, you should be able to set the front and back motor ports for each side equal to the same port

  constexpr int RAMP_MTR_PORT = 5;
  constexpr int INTAKE_LIFT_MTR_PORT = 1;
  constexpr int INTAKE_LEFT_MTR_PORT = 4;
  constexpr int INTAKE_RIGHT_MTR_PORT = 8;

  constexpr int IMU_PORT = 15;

  //LEDs
  constexpr int LED_1 = 1;
  constexpr int LED_2 = 1;
  constexpr int LED_3 = 1;
  constexpr int LED_4 = 1;

  //ultrasonic
  constexpr int FRONT_ULTRASONIC_ECHO_PORT = 8;
  constexpr int FRONT_ULTRASONIC_PING_PORT = 7;

  constexpr int BACK_ULTRASONIC_ECHO_PORT = 4;
  constexpr int BACK_ULTRASONIC_PING_PORT = 3;


}

namespace world_settings {
  constexpr float GRAVITY = 980; //cm
}



namespace callibrationSettings {

  const float DrivetrainKM = 0.01; //for APIDs. This is what we assume the conversion between volts and acceleration It should be porportional to how hard the robot is to accellerate

  constexpr double wheelRadius = 0.053; //radius of the wheels in meters
  constexpr double wheelBaseRadius = 0.17526; //dist from wheels to center of the robot (meters)

    //for back ultrasonic callibration (cm)
  //  constexpr float fieldSize = 365.76;
//    constexpr float distFromLine = 177.5;
//    constexpr float robotSize = 22.5;
  //  constexpr float flagDistFromWall = 27.5;
  //  constexpr float fronSensorToFrontOfCannonDist = 8;

    constexpr double TURN_CORRECTION = 750;
    constexpr double MOTOR_POSITION_ERROR = 20; //deviation in degrees from where the motor is set to be NOTE: if made too small the motor's accuraccy won't be abe to keep up and, among other things, the auton will be stuck trying to reach it''s set point forever
    //constexpr int CANNON_LAUNCH_ON_OFF_DELAY = 300;

}
#endif
