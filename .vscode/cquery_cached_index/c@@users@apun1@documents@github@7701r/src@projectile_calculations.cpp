//OUTDATED 2018

/*
#include "generalFunctions.h"
#include "Settings.h"

constexpr bool prj_cal_high_angle = false;  //for projectile calculations, whether to fire at the high angle or low angle (true = high, false = low)

//ultasonic Sensors
pros::ADIUltrasonic frontUltra (ports::FRONT_ULTRASONIC_PING_PORT, ports::FRONT_ULTRASONIC_ECHO_PORT);
pros::ADIUltrasonic backUltra (ports::BACK_ULTRASONIC_PING_PORT, ports::BACK_ULTRASONIC_ECHO_PORT);

float flagHeights[]{33.655,75,110};

int angleCalculationTimes = 1; //the number of times theta is recalculated with the new launchvelocity. Higher number increases accuraccy, but if using a non-variable launcher, it should always be set at 1.
double launchVelocity;
float g = world_settings::GRAVITY;



double fireAngle, launchAngle1, launchAngle2, recordedXDistFromTarg, xDistFromTarg,physicalYDistFromTarg,yDistFromTarg,lADeg,lADeg2;
//fireAngle is the angle currently being fired, launchAngleLow and launchAngleHigh are the 2 possible angles

float calculateAngle(); //declared below and should only be used in this file
double calculateSuperLaunchVelocity(double cannonAngle);

extern pros::ADIAnalogIn cannonLaunchVelocityCallibratorPot;
extern double cannonLaunchVelocity;
float getLaunchVelocity(float theta) {
  //return (0.0842*theta*theta-8.7804*theta+845.93); // for pneumatic launcher (experimental)
  //return calculateSuperLaunchVelocity(theta)*100; //for pneumatic launcher (theoretical)

  return  cannonLaunchVelocity; //for non-variable launcher
}

void autoFire(const int flag) {
  if (getXDistance() == ERROR) return;
  if (aim(flag)) { //aim will return false if it was unsuccessful aiming
    pros::delay(200);
    fire();
  }
}

extern pros::Motor cannonFireMotor;
void fire() { //cannon should be aimed first
  consoleLogN("-FIRING-");
  std::cout << "fire requested...\n";
  LEDs(false);
  double cFStartPos = cannonFireMotor.get_position();
  std::cout << "cFmtr start pos:" << cFStartPos << "\n";
  cannonFireMotor.move(200);
  while(fabs(cannonFireMotor.get_position()-cFStartPos) < 2*360) {
    pros::delay(1);
  }
  std::cout << "Firing completed. End cFmtr pos: " << cannonFireMotor.get_position() << "\n";
}

bool aim(const int flag) {
  pros::Motor aim_mtr(ports::CANNON_AIM_PORT,pros::E_MOTOR_GEARSET_18,1,pros::E_MOTOR_ENCODER_DEGREES);

  if (flag == 3) { //aiming at the barrelAngle
    physicalYDistFromTarg = xDistFromTarg * tan(aim_mtr.get_position() / 5); //calculating hight based on the distance in the x and the angle of the barrel (angMotor/5 because of gear ratios)
  }else{
    physicalYDistFromTarg = flagHeights[flag];
  }

  fireAngle = calculateAngle();

  if (fireAngle == ERROR){
    vibrateController("-.-");
    consoleLogN("ERROR: Beyond Range");
    return false;
  }else{
    consoleLogN("Firing At:");
    consoleLogN(fireAngle);
    consoleLogN("");

    std::cout << ("Aiming to: " + std::to_string(fireAngle) + ", Velocity: " + std::to_string(launchVelocity) + "cm/s") << "\n";
    pros::delay(10);
    setMotorPosition(aim_mtr,5 * fireAngle,100,0);
    return true;
  }
}


int findAimAngle(const int flag, int numCalculations) {
  extern pros::Motor aim_mtr;
  recordedXDistFromTarg = getXDistance();

  if (xDistFromTarg == ERROR) return ERROR;

  if (flag == 3) { //aiming at the barrelAngle
    physicalYDistFromTarg = xDistFromTarg * tan(aim_mtr.get_position() / 5); //calculating hight based on the distance in the x and the angle of the barrel (angMotor/5 because of gear ratios)
  }else{
    physicalYDistFromTarg = flagHeights[flag];
  }

  int prevAngleCalculationTimes_temp = angleCalculationTimes;
  angleCalculationTimes = numCalculations;

  fireAngle = calculateAngle(); //calculate the fire angle (degrees)

  angleCalculationTimes = prevAngleCalculationTimes_temp;

  if (fireAngle == ERROR){
    return ERROR;
  }else{
    return fireAngle;
  }
}

bool useFrontSensorForDistance = true;
float frontXDistFromTarg, backXDistFromTarg,xDist_Temp;

float getXDistance() {
  pros::delay(1);
  frontXDistFromTarg = ((float)frontUltra.get_value())/10 - callibrationSettings::flagDistFromWall + callibrationSettings::fronSensorToFrontOfCannonDist;
  pros::delay(1);
  backXDistFromTarg = callibrationSettings::fieldSize - callibrationSettings::robotSize - ((float)backUltra.get_value())/10 - callibrationSettings::flagDistFromWall;
  pros::delay(1);

  if ((fabs(backXDistFromTarg - (callibrationSettings::fieldSize - callibrationSettings::robotSize - callibrationSettings::flagDistFromWall) ) <= 1)) {
    xDist_Temp = frontXDistFromTarg;
  }else if ( useFrontSensorForDistance) {
    xDist_Temp = frontXDistFromTarg;
  }else{
    xDist_Temp = backXDistFromTarg;
  }
  std::cout << "xDistFromTarg: " << xDist_Temp << "\n";

  consoleLogN("frontUltra: " + std::to_string(frontXDistFromTarg));
  consoleLogN("backUltra: " + std::to_string(backXDistFromTarg));
  if ((xDist_Temp <= 0) || (xDist_Temp > callibrationSettings::fieldSize)) {
    consoleLogN("Error when calculating distance (" + std::to_string(xDist_Temp) + ") cm");
    return ERROR;
  }else{
    xDistFromTarg = xDist_Temp;
    consoleLogN("Target is " + std::to_string(xDistFromTarg) + "cm away");
    return xDistFromTarg;
  }

}

void setXDistFromTarget(float xDist1) {
  xDistFromTarg = xDist1;
}

extern double Lc,Hc;
float calculateAngle() {
  bool Error = true;

  fireAngle = 45;

  for (int i = 0; i < angleCalculationTimes; i++) {

    xDistFromTarg = recordedXDistFromTarg + (Lc-Lc*cos(fireAngle))*100;
    yDistFromTarg = physicalYDistFromTarg - (Lc*sin(fireAngle) + Hc)*100;
    launchVelocity = getLaunchVelocity(fabs(fireAngle));
    if (launchVelocity == ERROR) return ERROR;
    double xF = xDistFromTarg,Vi = launchVelocity, yF = yDistFromTarg;

    double a = ( g * xF * xF) / (2 * Vi * Vi);

  	if ((xF*xF - 4 * a * (yF + a)) < 0) {
          Error = true;

          if (prj_cal_high_angle) {
            fireAngle+= 40/angleCalculationTimes;
          }else{
            fireAngle-= 40/angleCalculationTimes;
          }
          continue;
    }else{
  			Error = false;
  	}

  	launchAngle1 = -180 * atan( (-xF + sqrt(xF*xF - 4 * a * (yF + a))) / (2*a) ) / PI;
  	launchAngle2 = -180 * atan( (-xF - sqrt(xF*xF - 4 * a * (yF + a))) / (2*a) ) / PI;

    std::cout << "launch angle (1): " << launchAngle1  << "\n";
    std::cout << "launch angle (2): " << launchAngle1  << "\n";

    if (prj_cal_high_angle) {
      fireAngle = launchAngle1 > launchAngle2 ? launchAngle1 : launchAngle2;
    }else{
      fireAngle = launchAngle1 < launchAngle2 ? launchAngle1 : launchAngle2;;
    }
  }



  if (Error) {
    return ERROR;
  }else{
    return fireAngle;
  }
}

*/
