#include "generalFunctions.h"
#include "Settings.h"

//motor declaration (declaraton is in initialize.cpp)
  //wheels
 extern GEAH::Motor left_mtr_back;
 extern GEAH::Motor right_mtr_back;

 extern GEAH::Motor left_mtr_front;
 extern GEAH::Motor right_mtr_front;

 extern GEAH::Motor ramp_mtr;
 extern GEAH::Motor intake_lift_mtr;
 extern GEAH::Motor left_intake;
 extern GEAH::Motor right_intake;


class realTimePositionController;

extern std::vector<realTimePositionController*> realTimePositionControllers;

class realTimePositionController {
  private:
    GEAH::Motor* motorToControl;
    std::string name = "";
    bool isActivated = true;

    float speedModifier = 1;
    bool modulated = false;
    int positionType = MOVE_DEGREES;
    int direction = 0; //-1 = only backwards, 1 = only forwards, 0 = either forwards or backwards

    bool PID;
    float porportion, integral, derivative;

    int motorSystemRadius = callibrationSettings::wheelRadius;
    int cannotMoveCounter = 0;
    float angleDelta = 0;
    double targetPosition = ERROR;
    float moveVelocity = 0;
    int motorPowerIncreaser = 0;

  private:
    double getMotorPosition() {
      double deg_value =(double) (*motorToControl).get_position();
      if (modulated) deg_value = (double)  (((int)deg_value) % 360);

      switch(positionType) {

        case(MOVE_ROTATIONS):
          return deg_value/360;
        break;

        case(MOVE_METERS):
          return motorSystemRadius * deg_value * PI / 180;
        break;

        case(MOVE_DEGREES):
          return deg_value;
          break;
        default:
          consoleLogN("ERROR in APID " + name + ": invaid positionType");
          return ERROR;
          break;
      };

    }

  public:
    realTimePositionController(GEAH::Motor* motorToControl1, std::string name1) : motorToControl{motorToControl1}, name{name1} {
      PID = false;
    }

    realTimePositionController(GEAH::Motor* motorToControl1, std::string name1,float porportion1, float integral1, float derivative1) : motorToControl{motorToControl1}, name{name1}, porportion{porportion1}, integral{integral1}, derivative{derivative1} {
      PID = true;
    }

    void initialize() {
      if (PID) {
        pros::motor_pid_full_s_t pid = pros::Motor::convert_pid_full(0,porportion, integral, derivative,1,1,callibrationSettings::MOTOR_POSITION_ERROR/2,10);
        (*motorToControl).set_pos_pid_full(pid);
      }
    }

    GEAH::Motor* getMotorPointer() {
      return motorToControl;
    }

    void disable() {
      if (PID) {
        pros::motor_pid_s_t pid = pros::Motor::convert_pid(0,0,0,0);
        (*motorToControl).set_pos_pid(pid);
      }
    }
    void run() {
      if (PID) {
        if (isActivated) {
          if (targetPosition != ERROR) (*motorToControl).move_absolute(targetPosition, 63.5 * speedModifier);
        }
      }else{
        pros::delay(1);
        if ((moveVelocity > 0.001) && ((*motorToControl).get_actual_velocity() == 0)) {
          cannotMoveCounter++;
        }else{
          cannotMoveCounter = 0;
        }

        if (cannotMoveCounter > 20) {
          consoleLogN("aim motor cannot move. Terminating movement...");
          std::cout << ("aim motor cannot move. Terminating movement...");
          moveVelocity = 0;
          (*motorToControl).move_velocity(0);
        }else if (isActivated) {
          if (modulated) {
            angleDelta =  ((int)targetPosition % 360) - getMotorPosition();
          }else{
            angleDelta = targetPosition - getMotorPosition();
          }
          pros::delay(1);
          moveVelocity = fabs(angleDelta)/angleDelta * (motorPowerIncreaser + speedModifier * (0.7*fabs(angleDelta)+3*sqrt(fabs(angleDelta))-3*pow(fabs(angleDelta),1/4)+.322496));
          if (direction == 0) {
            (*motorToControl).move_velocity(moveVelocity);
          }else{
            if (direction == 1) {
              if (moveVelocity >= 0) {
                (*motorToControl).move_velocity(moveVelocity);
              }else{
                (*motorToControl).move_velocity(0);
              }
            }else if (direction == -1) {
              if (moveVelocity <= 0) {
                (*motorToControl).move_velocity(moveVelocity);
              }else{
                (*motorToControl).move_velocity(0);
              }
            }else{
              consoleLogN("ERROR in APID: " + name + " -- invalid direction");
              std::cout << ("ERROR in APID: " + name + " -- invalid direction (direction:") << direction << ")\n";
            }
          }
          if ((fabs((*motorToControl).get_actual_velocity()) < 0.5 * fabs(moveVelocity)) && (fabs(moveVelocity) > 0.1)) {
            motorPowerIncreaser++;
          }else{
            if (motorPowerIncreaser > 0) motorPowerIncreaser--;
          }
        }else{
          moveVelocity = 0;
        }
      }
  }
  void setSpeedModifier(float speedModifier1) {
    speedModifier = speedModifier1;
    run();
  }
  std::string getName() {
    return name;
  }

  double getTargetPosition() {
    return targetPosition;
  }

  bool getIsActivated() {
    return isActivated;
  }

  void setMotorSystemRadius(double newMotorSystemRadius) {
    motorSystemRadius = newMotorSystemRadius;
  }
  void setTargetPosition(double newPos) {
    targetPosition = newPos;
    run();
  }

  void setIsModulated(bool mod) {
    modulated = mod;
  }

  void setDirection(int dir) {
    direction = dir;
  }

  void setIsActivated(bool isActive1) {
    isActivated = isActive1;
    if ((isActive1) && (! isActivated)) {
      initialize();
      run();
    }else if ((! isActive1) && (isActivated)){
      disable();
    }


  }

  void setPositionType(int newPosType){
    positionType = newPosType;
  }


};




//drivePIDs
const float drivePorportion = 0.7f;
const float driveIntegral = 0.01f;
const float driveDerivative = 0.02f;
realTimePositionController left_mtr_back_PID{&left_mtr_back,"lb_drive_PID",drivePorportion,driveIntegral,driveDerivative};
realTimePositionController right_mtr_back_PID{&right_mtr_back,"rb_drive_PID",drivePorportion,driveIntegral,driveDerivative};
realTimePositionController left_mtr_front_PID{&left_mtr_front,"lf_drive_PID",drivePorportion,driveIntegral,driveDerivative};
realTimePositionController right_mtr_front_PID{&right_mtr_front,"rf_drive_PID",drivePorportion,driveIntegral,driveDerivative};

realTimePositionController intake_lift_PID{&intake_lift_mtr,"intake_lift_PID",1.5f,1.0f,0.01f};
realTimePositionController ramp_PID{&ramp_mtr,"ramp_PID",10.0f,0.01f,0.30f};

realTimePositionController left_intake_PID{&left_intake,"left_intake_PID",0.5f,0.01f,0.0f};
realTimePositionController right_intake_PID{&right_intake,"right_intake_PID",0.5f,0.01f,0.0f};




std::vector<realTimePositionController*> realTimePositionControllers = {&left_mtr_back_PID,&right_mtr_back_PID,&right_mtr_front_PID,&left_mtr_front_PID,&intake_lift_PID,&ramp_PID,&left_intake_PID,&right_intake_PID};

void initializeAutoPilot() {

  for (realTimePositionController *RTPS : realTimePositionControllers) {
    std::cout << "initializing " << (*RTPS).getName() << "\n";
    (*RTPS).initialize();
  }



}

realTimePositionController* getRealTimePositionController(std::string name) {
//  std::cout << "Searching for " + name + " - ";
  pros::delay(10);
  for (realTimePositionController *RTPS : realTimePositionControllers) {
    realTimePositionController RTPSS = *RTPS;
    if (RTPSS.getName() == name) {
      return RTPS;
    }
  }
  pros::delay(10);
  consoleLogN("ERROR: realTimePositionController " + name + " was not found");
  pros::delay(10);
  std::cout << "ERROR: realTimePositionController " + name + " was not found" << "\n";
  pros::delay(10);
  return NULL;
}



void setDriveTrainTarget(double newTarg,double speed) {
  double speedModifier = speed/63.5;
  setAPIDTarget("rb_drive_PID",newTarg);
  setAPIDTarget("lb_drive_PID",newTarg);
  setAPIDTarget("lf_drive_PID",newTarg);
  setAPIDTarget("rf_drive_PID",newTarg);
  (*getRealTimePositionController("lb_drive_PID")).setSpeedModifier(speedModifier);
  (*getRealTimePositionController("rb_drive_PID")).setSpeedModifier(speedModifier);
  (*getRealTimePositionController("rf_drive_PID")).setSpeedModifier(speedModifier);
  (*getRealTimePositionController("lf_drive_PID")).setSpeedModifier(speedModifier);
}

void setLeftDriveTrainTarget(double newTarg,double speed) {
  double speedModifier = speed/63.5;
  setAPIDTarget("lb_drive_PID",newTarg);
  setAPIDTarget("lf_drive_PID",newTarg);
  (*getRealTimePositionController("lb_drive_PID")).setSpeedModifier(speedModifier);
  (*getRealTimePositionController("lf_drive_PID")).setSpeedModifier(speedModifier);
}

void setRightDriveTrainTarget(double newTarg,double speed) {;
  double speedModifier = speed/63.5;
  setAPIDTarget("rb_drive_PID",newTarg);
  setAPIDTarget("rf_drive_PID",newTarg);
  (*getRealTimePositionController("rb_drive_PID")).setSpeedModifier(speedModifier);
  (*getRealTimePositionController("rf_drive_PID")).setSpeedModifier(speedModifier);
}

void setDriveTrainPIDIsActivated(bool isActivated) {
  setAPIDIsActivated("rb_drive_PID",isActivated);
  setAPIDIsActivated("lb_drive_PID",isActivated);
  setAPIDIsActivated("lf_drive_PID",isActivated);
  setAPIDIsActivated("rf_drive_PID",isActivated);

}

void setIntakeAPIDIsActivated(bool isActivated){
  setAPIDIsActivated("left_intake_PID", isActivated);
  setAPIDIsActivated("right_intake_PID", isActivated);
}

void setIntakeAPIDTargetAndSpeed(double targPos, double speed) {
  (*getRealTimePositionController("left_intake_PID")).setTargetPosition(targPos);
  (*getRealTimePositionController("left_intake_PID")).setSpeedModifier(speed/63.5);
  (*getRealTimePositionController("right_intake_PID")).setTargetPosition(targPos);
  (*getRealTimePositionController("right_intake_PID")).setSpeedModifier(speed/63.5);
}

double getAPIDTarget(std::string apidName) {
  return (*getRealTimePositionController(apidName)).getTargetPosition();
}

void setAPIDTarget(std::string apidName, double targPos) {
  (*getRealTimePositionController(apidName)).setTargetPosition(targPos);
}

void setAPIDTargetAndSpeed(std::string apidName, double targPos, double speed) {
  (*getRealTimePositionController(apidName)).setTargetPosition(targPos);
  (*getRealTimePositionController(apidName)).setSpeedModifier(speed/63.5);
}

void setAPIDIsActivated(std::string apidName, bool isActivated) {
    (*getRealTimePositionController(apidName)).setIsActivated(isActivated);
}

GEAH::Motor getAPIDMotor(std::string apidName) {
  return *((*getRealTimePositionController(apidName)).getMotorPointer());
}
void autoPilotController(long loops) {

  if (loops % 30 == 0) {
    if (ramp_mtr.get_temperature() >= 55) {
      consoleLogN("ramp_mtr is overheated!");
      std::cout << "ramp_mtr is overheated!" << ramp_mtr.get_temperature() << "\n";
      setAPIDIsActivated("ramp_PID", false);
    }
    if (intake_lift_mtr.get_temperature() >= 55) {
      consoleLogN("intake_lift_mtr is overheated!");
      std::cout << "intake_lift_mtr is overheated!" << intake_lift_mtr.get_temperature() << "\n";
      setAPIDIsActivated("intake_lift_PID", false);
    }

    if (left_intake.get_temperature() >= 55) {
      consoleLogN("left_intake is overheated!");
      std::cout << "left_intake is overheated!" << left_intake.get_temperature() << "\n";
    }
    if (right_intake.get_temperature() >= 55) {
      consoleLogN("right_intake is overheated!");
      std::cout << "right_intake is overheated!" << right_intake.get_temperature() << "\n";
    }
  }

/*
  for (realTimePositionController *RTPS : realTimePositionControllers) {
    (*RTPS).run();
  }
*/
  if (   (*getRealTimePositionController("lb_drive_PID")).getIsActivated()) { //if drive Pids are activated
    left_mtr_front.move_velocity(left_mtr_back.get_target_velocity());
    right_mtr_front.move_velocity(right_mtr_back.get_target_velocity());
  }
}
