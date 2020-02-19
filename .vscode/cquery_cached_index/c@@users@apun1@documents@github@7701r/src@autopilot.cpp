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

    double speedModifier = 1;
    bool modulated = false;
    int positionType = MOVE_DEGREES;

    float porportion, integral, derivative;

    int motorSystemRadius = callibrationSettings::wheelRadius;
    int cannotMoveCounter = 0;
    float angleDelta = 0;
    double targetPosition = ERROR;
    float moveVelocity = 0;
    int motorPowerIncreaser = 0;



  public:
    realTimePositionController(GEAH::Motor* motorToControl1, std::string name1) : motorToControl{motorToControl1}, name{name1} {
      consoleLogN("ERROR - NON-PID APID CONSTRUCTOR SHOULD NEVER BE USED!");
      std::cout << "ERROR - NON-PID APID CONSTRUCTOR SHOULD NEVER BE USED!" << std::endl;
    }

    realTimePositionController(GEAH::Motor* motorToControl1, std::string name1,float porportion1, float integral1, float derivative1) : motorToControl{motorToControl1}, name{name1}, porportion{porportion1}, integral{integral1}, derivative{derivative1} {}

    void initialize() {
      pros::motor_pid_full_s_t pid = pros::Motor::convert_pid_full(0,porportion, integral, derivative,1,1,callibrationSettings::MOTOR_POSITION_ERROR/2,10);
      (*motorToControl).set_pos_pid_full(pid);
      (*motorToControl).setAPIDConstants(porportion, integral, derivative);
      (*motorToControl).setSpeedModifier(&speedModifier);
    }

    GEAH::Motor* getMotorPointer() {
      return motorToControl;
    }

    void disable() {
      pros::motor_pid_s_t pid = pros::Motor::convert_pid(0,0,0,0);
      (*motorToControl).set_pos_pid(pid);
    }
    void run() {
      if (isActivated) {
        if (targetPosition != ERROR) (*motorToControl).runPid();
      }
  }
  void setSpeedModifier(double speedModifier1) {
    speedModifier = std::abs(speedModifier1);
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
    (*motorToControl).setAPIDTarget(newPos);
  }

  void setIsActivated(bool isActive1) {

    if ((isActive1) && (! isActivated)) {
      initialize();
    }else if ((! isActive1) && (isActivated)){
      disable();
    }

    isActivated = isActive1; //do this after so we can use isActivated as a prev var for the if statements
  }

  void setPositionType(int newPosType){
    positionType = newPosType;
  }


};




//drivePIDs
const float drivePorportion = 0.3f;
const float driveIntegral = 0.02f;
const float driveDerivative = 0.01f;
realTimePositionController left_mtr_back_PID{&left_mtr_back,"lb_drive_PID",drivePorportion,driveIntegral,driveDerivative};
realTimePositionController right_mtr_back_PID{&right_mtr_back,"rb_drive_PID",drivePorportion,driveIntegral,driveDerivative};
realTimePositionController left_mtr_front_PID{&left_mtr_front,"lf_drive_PID",drivePorportion,driveIntegral,driveDerivative};
realTimePositionController right_mtr_front_PID{&right_mtr_front,"rf_drive_PID",drivePorportion,driveIntegral,driveDerivative};

realTimePositionController intake_lift_PID{&intake_lift_mtr,"intake_lift_PID",1.5f,1.0f,0.01f};
realTimePositionController ramp_PID{&ramp_mtr,"ramp_PID",1.9f,0.01f,0.4f};





std::vector<realTimePositionController*> realTimePositionControllers = {&left_mtr_back_PID,&right_mtr_back_PID,&right_mtr_front_PID,&left_mtr_front_PID,&intake_lift_PID,&ramp_PID};

void initializeAutoPilot() {

  for (realTimePositionController *RTPS : realTimePositionControllers) {
    std::cout << "initializing " << (*RTPS).getName() << "\n";
    (*RTPS).initialize();
    (*RTPS).setIsActivated(true);
    (*RTPS).setIsActivated(false);
  }
}

realTimePositionController* getRealTimePositionController(std::string name) {
///  std::cout << "Searching for " + name + " - ";
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
  std::cout << "ERROR: realTimePositionController " + name + " was not found. " << " Memory permission error will be thrown.";
  pros::delay(10);
  return NULL;
}



void setDriveTrainTarget(double newTarg,double speed) {
  double speedModifier = speed;
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
  double speedModifier = speed;
  std::cout << "set lb_drive_PID  targ" << std::endl;
  pros::delay(10);
  setAPIDTarget("lb_drive_PID",newTarg);
  setAPIDTarget("lf_drive_PID",newTarg);

  std::cout << "set speed modifier" << std::endl;
  (*getRealTimePositionController("lb_drive_PID")).setSpeedModifier(speedModifier);

  (*getRealTimePositionController("lf_drive_PID")).setSpeedModifier(speedModifier);
}

void setRightDriveTrainTarget(double newTarg,double speed) {;
  double speedModifier = speed;
  setAPIDTarget("rb_drive_PID",newTarg);
  setAPIDTarget("rf_drive_PID",newTarg);
  (*getRealTimePositionController("rb_drive_PID")).setSpeedModifier(speedModifier);
  (*getRealTimePositionController("rf_drive_PID")).setSpeedModifier(speedModifier);
}

void setLeftDriveTrainPIDSpeedModifier(double newSpeed) {
  (*getRealTimePositionController("rb_drive_PID")).setSpeedModifier(newSpeed);
  (*getRealTimePositionController("rf_drive_PID")).setSpeedModifier(newSpeed);
}

void setRightDriveTrainPIDSpeedModifier(double newSpeed) {
  (*getRealTimePositionController("lb_drive_PID")).setSpeedModifier(newSpeed);
  (*getRealTimePositionController("lf_drive_PID")).setSpeedModifier(newSpeed);
}

void setDriveTrainPIDIsActivated(bool isActivated) {
  setAPIDIsActivated("rb_drive_PID",isActivated);
  setAPIDIsActivated("lb_drive_PID",isActivated);
  setAPIDIsActivated("lf_drive_PID",isActivated);
  setAPIDIsActivated("rf_drive_PID",isActivated);

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

void runPIDs() {
  for (realTimePositionController *RTPS : realTimePositionControllers) {
    (*RTPS).run();
  }
}

void autoPilotController(long loops) {

  runPIDs();



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

  if (   (*getRealTimePositionController("lb_drive_PID")).getIsActivated()) { //if drive Pids are activated
    left_mtr_front.move_velocity(left_mtr_back.get_actual_velocity());
    right_mtr_front.move_velocity(right_mtr_back.get_actual_velocity());
  }
}
