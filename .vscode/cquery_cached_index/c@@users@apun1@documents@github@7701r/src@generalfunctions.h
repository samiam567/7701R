#ifndef GENERAL_FUNCTIONS_H
#define GENERAL_FUNCTIONS_H
#include "main.h"


//position/movement types
constexpr int MOVE_METERS = 0;
constexpr int MOVE_ROTATIONS = 1;
constexpr int MOVE_DEGREES = 2;

int getAuton(); //returns the id of the selected auton (in initialize.cpp)
int getAuton(std::string auton_name); //returns the id of the auton with the passed name (in initialize.cpp)
std::string getAutonName(int auton_sel);

void LEDs(bool state); //controls the LEDs (in initalize.cpp)

bool moveMotor(pros::Motor motor, float magnatude, int speed, int type);
bool setMotorPosition(pros::Motor motor, float magnatude, int speed, int type);

void autonomous();
void autonomous(int auton_sel);

bool checkForStop();

int getDrive();

int getAuton(); //(in initialize.cpp)


void vibrateController(const char*rumble_pattern); //(in opcontroll.cpp)


namespace GEAH {

  class Motor : public pros::Motor {
    private:
       std::string name = "unNamed";
       double preCallibrationPos = 0; //for autonCallibration
    public:


      Motor(std::string Name, const std::uint8_t port, const pros::motor_gearset_e_t gearset, const std::uint8_t isReversed, const pros::motor_encoder_units_e_t encoderMode) : pros::Motor(port,gearset,isReversed,encoderMode) {
        name = Name;
        std::cout << name << " has been created \n";
      }

      void operator= (int voltage ) {
        move(voltage);
      }

      void operator= (double voltage ) {
        move(voltage);
      }
      void operator= (long voltage ) {
        move(voltage);
      }

      void setPreCallibrationPos(double preCalPos) {
        preCallibrationPos = preCalPos;
      }

      double getPreCallibrationPos() {
        return preCallibrationPos;
      }

      std::string getName() {
        return name;
      }
  };

  class autonRequestReceipt {
    public:
      int id;
      std::string requestType;
      double magnatude;
      int speed, type;
      std::vector<GEAH::Motor> motors;

      autonRequestReceipt(std::string requestType1) : requestType{requestType1} {}
      autonRequestReceipt(std::string requestType1, std::vector<GEAH::Motor> motors1, double magnatude1, int speed1, int type1) : requestType{requestType1}, motors{motors1}, magnatude{magnatude1}, speed{speed1}, type{type1} {}
  };

  struct controllerButton {
    pros::controller_digital_e_t baseButton;
    int controller;
    bool shift;
    std::string name;
  };

  struct driver {
    std::string name;
    int drive_type; //0 = arcade, 1 = tank, 2 = drone
    GEAH::controllerButton shiftButton; //this will change all the buttons to their shift states(which doubles the number of funtion possibilities)
    GEAH::controllerButton ramp_up, ramp_down;
    GEAH::controllerButton intake_up, intake_down;
    GEAH::controllerButton intake_in, intake_out;
    GEAH::controllerButton lockWheelsIntake;
    GEAH::controllerButton LAUNCH_AUTON;
    GEAH::controllerButton BREAKS_ON,BREAKS_OFF;
    //GEAH::controllerButton auto_unload;


    int LED_CONFIG;
  };
}

//projectile functions
void fire(); //fire the cannon (projectile_calculations.cpp)
void autoFire(const int flag); //autoFire the cannon (projectile_calculations.cpp)
float getXDistance(); //get x distance from flag with ultrasonic sensor (projectile_calculations.cpp)
void setXDistFromTarget(float xDist1); //set the x distance from the flag
bool aim(const int flag); //aim the barrel at the specified flag (note: aim will not get the xDistance by itself and getXDistance() or setXDistance(float xDist1) must be called first)
int findAimAngle(const int flag, int numCalculations); //finds the angle of the barrel to fire at the given flag. Basically the same as aim(const int flag) except this function will return the degree value instead of physically aiming the barrel.
float getXDistance();

GEAH::driver getDriver(); //(in initialize.cpp)

//autoPilot (in autoPilot.cpp)
void autoPilotController(long loops);
void setTargetCannonAngle(double theta);
void setAPIDTarget(std::string apidName, double targPos);
void setAPIDIsActivated(std::string apidName, bool isActivated);
double getAPIDTarget(std::string apidName);

void setDriveTrainTarget(double newTarg, double speed);
void setLeftDriveTrainTarget(double newTarg, double speed);
void setRightDriveTrainTarget(double newTarg, double speed);

void setDriveTrainPIDIsActivated(bool isActivated);

//console (in Graphics.cpp)
void consoleLog(std::string data);
void consoleLog(int data);
void consoleLog(double data);
void consoleLog(float data);
void consoleLog(char data);
void consoleLogN(std::string data);
void consoleLogN(int data);
void consoleLogN(double data);
void consoleLogN(float data);
void consoleLogN(char data);
void consoleClear();

//robotCodeToolbox (in robotCodeToolbox.cpp)
std::string remoteSettingSelect(std::string title, std::vector<std::string> choices, std::string outputType);


//robotCallibration (in robotCallibration.cpp)
void recordRobotAutonMovement(GEAH::autonRequestReceipt receipt); //<- used for odometry and auton callibration

//opControl (in opcontrol.cpp)
void runOpControl();
#endif
