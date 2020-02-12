#ifndef GEAH_MOTOR_H_EXISTS
#define GEAH_MOTOR_H_EXISTS

#include "main.h"


namespace GEAH {
  class Motor : public pros::Motor {
    private:
       std::string name = "unNamed";

    //for APIDs
       float kP, kI, kD; //the constants for the pid
       double pidTarget;
       double prevVelocity;
       long prevTime;
       double* speedModifier;
    //end APID vars

       double preCallibrationPos = 0; //for autonCallibration
    public:

      Motor(std::string Name, const std::uint8_t port, const pros::motor_gearset_e_t gearset, const std::uint8_t isReversed, const pros::motor_encoder_units_e_t encoderMode);

      void operator= (int voltage );

      void operator= (double voltage );

      void operator= (long voltage );

      void setPreCallibrationPos(double preCalPos);

      double getPreCallibrationPos();

      void setSpeedModifier(double* newSpeedModifier);
      void setAPIDTarget(double targ);
      void setAPIDConstants(double nkP, double nkI, double nkD);

      void runPid();

      std::string getName();
  };
}

#endif
