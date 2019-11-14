#include "generalFunctions.h"
#include "graphics.h"
#include "Settings.h"

//motor declaration (declaraton is in initialize.cpp)
  //wheels
 extern GEAH::Motor left_mtr_back;
 extern GEAH::Motor right_mtr_back;

 extern GEAH::Motor left_mtr_front;
 extern GEAH::Motor right_mtr_front;



 //controllers
 extern pros::Controller master, partner;


bool isCallibratingAuton = false;


std::vector<std::string> callibrationTypes = {"autonCallibration"};

extern std::vector<std::string> autonNames;

bool runCallibration() {
  initializeLVGL();
  std::string callibrationMode = remoteSettingSelect("callibration mode",callibrationTypes,"console");
  consoleLogN("Running " + callibrationMode);

  if (callibrationMode == "autonCallibration") {
    isCallibratingAuton = true;
    autonomous(getAuton(remoteSettingSelect("Auton to test", autonNames,"console")));
    isCallibratingAuton = false;
  }else{
    std::cout << "Error in runCallibration(): invalid callibrationMode";
    consoleLogN("Error in runCallibration(): invalid callibrationMode");
  }
  return true;
}


extern std::vector<GEAH::Motor> Motors;
//used for odometry and auton callibration
void recordRobotAutonMovement(GEAH::autonRequestReceipt receipt) {
  if (isCallibratingAuton) {
    pros::delay(10);

    //record position before
    for (GEAH::Motor cMotor : receipt.motors.size() > 0 ? receipt.motors : Motors) {
      cMotor.setPreCallibrationPos(cMotor.get_position());
    }

    consoleLogN("Now use the controller to adjust the position.");
    std::cout << "correcting " << receipt.requestType << "\n";
    for (int times = 1000; times >= 0; times--) {
      runOpControl();
      if ((times % 200) == 0) std::cout << times << " loops left for correction \n";
    }

    //autput adjustments
    consoleClear();
    double adjustment;
    for (GEAH::Motor cMotor : receipt.motors.size() > 0 ? receipt.motors : Motors) {
      pros::delay(10);
      adjustment = cMotor.get_position() - cMotor.getPreCallibrationPos();
      consoleLog(cMotor.getName() + ": ");
      consoleLogN(adjustment);
      std::cout << cMotor.getName() << " has adjusted " << adjustment << " units." << "\n";
    }
    pros::delay(100);

    consoleLogN("Press A to continue");
    while (! master.get_digital(DIGITAL_A)) pros::delay(30);
    pros::delay(500);

  }
}
