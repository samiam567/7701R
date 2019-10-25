#include "generalFunctions.h"
#include "Settings.h"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

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


double lTarget = ERROR, rTarget = ERROR;
int aTLoops = 0;
bool checkForStop() {
  return false;
}

bool usePIDForDriveTrainAutonMovement = true;



bool driveStraight(double magnatude, int speed, int type) {
		//for type:
		//0 = rotations
		//1 = meters
    std::cout << "driveStraight called for magn " << magnatude << ", speed " << speed << ", and type " << type << "\n";
    float give = 0.1f;

		double wheelRotations;

		if (type == MOVE_ROTATIONS) {
			wheelRotations = magnatude;
		}else if (type == MOVE_METERS) {
			wheelRotations = magnatude / ( 2 * callibrationSettings::wheelRadius* M_PI);
      std::cout << "wheelRotations for move_meters: " << wheelRotations << "\n";
		}else if (type == MOVE_DEGREES) {
      wheelRotations = magnatude / 360;
    }else{
      std::cout << "invalide movement type for driveStraight. Defaulting to meters" << "\n";
			//default to distance
			wheelRotations = magnatude / ( 2 * callibrationSettings::wheelRadius * M_PI);
		}

	   speed = fabs(speed);

     if (magnatude < 0) speed = -speed; //drive backwards if ordered to move negative distance



		double lValue,lStart;
		double rValue,rStart;
		lValue = left_mtr_back.get_position();
    rValue = right_mtr_back.get_position();

    if ((lTarget == ERROR) || (rTarget == ERROR)) {
      lStart = left_mtr_back.get_position();
      rStart = right_mtr_back.get_position();
    }else{
      lStart = lTarget;
      rStart = rTarget;

      //increment the target values with this request
      lTarget += wheelRotations*360;
      rTarget += wheelRotations*360;
    }

		float correctionValue;

  if (usePIDForDriveTrainAutonMovement) {
      setDriveTrainPIDIsActivated(false);
      float wheelDegrees = wheelRotations*360;

      std::cout << "PID driving " << wheelDegrees << " degrees\n";
      std::cout << "lStart: " << lStart << "\n";
      std::cout << "rStart: " << rStart << "\n";

      setLeftDriveTrainTarget(wheelDegrees + lStart,speed);
      setRightDriveTrainTarget(wheelDegrees + rStart,speed);

      std::cout << "driveTrainTarget: " << wheelDegrees + lStart << " degs \n";

      autoPilotController(0);
      int counter = 0;

      setDriveTrainPIDIsActivated(true);

      double lSpeed = speed, rSpeed = speed;
      while ( (fabs( lValue - (wheelDegrees+lStart)) > callibrationSettings::MOTOR_POSITION_ERROR) || ( fabs( rValue - (wheelDegrees+rStart)) > callibrationSettings::MOTOR_POSITION_ERROR)){
        lValue = left_mtr_back.get_position();
        rValue = right_mtr_back.get_position();


        double vDiff = fabs(left_mtr_back.get_actual_velocity()) - fabs(right_mtr_back.get_actual_velocity());

        lSpeed = speed + vDiff;
        rSpeed = speed - vDiff;


        setLeftDriveTrainTarget(wheelDegrees + lStart, lSpeed);
        setRightDriveTrainTarget(wheelDegrees + rStart,rSpeed);

/*
        //turn correction
        if ( (fabs(lValue-lStart) - fabs(rValue-rStart)) < 0.5*callibrationSettings::MOTOR_POSITION_ERROR) {
          double turn_error = fabs(lValue-lStart) - fabs(rValue-rStart);
          if (turn_error < 0) { //left is falling behind.. slow down right
            turn_error = fabs(turn_error);
            setLeftDriveTrainTarget(wheelDegrees + lStart,speed + turn_error*0.2/callibrationSettings::TURN_CORRECTION);
            setRightDriveTrainTarget(wheelDegrees + rStart,speed - turn_error*0.2/callibrationSettings::TURN_CORRECTION);
          }else{//right is falling behind.. slow down left
            turn_error = fabs(turn_error);
            setLeftDriveTrainTarget(wheelDegrees + lStart,speed - turn_error*0.2/callibrationSettings::TURN_CORRECTION);
            setRightDriveTrainTarget(wheelDegrees + rStart,speed + turn_error*0.2/callibrationSettings::TURN_CORRECTION);
          }
        }else{
          setRightDriveTrainTarget(wheelDegrees + rStart,speed);
          setLeftDriveTrainTarget(wheelDegrees + lStart,speed);
        }
*/

        if (counter %500==0) std::cout << "driving straight\nlValue: " << lValue << "\nrValue: " << rValue << "\ncounter:" << counter << "\n--\n";
        pros::delay(5);
        autoPilotController(counter);
        counter++;
        checkForStop();

        if (counter > fabs(10 + 10 * speed * wheelDegrees/M_PI/100) ) {
          consoleLogN("-ERROR- Turn taking too long. Terminating...");
          std::cout << "-ERROR- DriveStraight of magn: " << wheelDegrees << "degs and speed: " << speed << " taking too long. (counter = " << counter << ") \n" << "terminating turn... \n";
          break;
        }

      }

      setDriveTrainPIDIsActivated(false);
      left_mtr_back.move(0);
      right_mtr_back .move(0);
      left_mtr_front .move(0);
      right_mtr_front .move(0);

      pros::motor_brake_mode_e_t prevBrakeMode = left_mtr_back.get_brake_mode();

      left_mtr_back.set_brake_mode(MOTOR_BRAKE_HOLD);
      right_mtr_back.set_brake_mode(MOTOR_BRAKE_HOLD);
      right_mtr_front.set_brake_mode(MOTOR_BRAKE_HOLD);
      left_mtr_front.set_brake_mode(MOTOR_BRAKE_HOLD);
      pros::delay(100);
      left_mtr_back.set_brake_mode(prevBrakeMode);
      right_mtr_back.set_brake_mode(prevBrakeMode);
      right_mtr_front.set_brake_mode(prevBrakeMode);
      left_mtr_front.set_brake_mode(prevBrakeMode);


      std::cout << "finalPos:\n" << lValue << "\n" << rValue << "\n--\n";

      std::vector<GEAH::Motor>motors{left_mtr_back,right_mtr_back,left_mtr_front,right_mtr_front};
      GEAH::autonRequestReceipt receipt("driveStraight",motors,magnatude,speed,type);
      recordRobotAutonMovement(receipt); //<- used for odometry and auton callibration
      return true;

    }else{

    float degs = fabs(360*wheelRotations);

    int motorCannotMoveCounter = 0;

    while (lValue < degs ) {


      checkForStop(); //this method should be continuously called the entire duration of the program

      lValue = fabs(left_mtr_back.get_position()-lStart);
      rValue = fabs(right_mtr_back.get_position()-rStart);

      if (fabs(left_mtr_back.get_actual_velocity()) <= 0.1) motorCannotMoveCounter++;

      if (motorCannotMoveCounter > 2000) {
        consoleLogN("Error:: motor cannot move");
        consoleLogN("Turn Terminated");
        return false; //motor movement was in some way inhibited
      }

      if ( fabs(lValue - rValue) < give) { //the sides are equal and no correction needs to be made

        left_mtr_back = speed;
        left_mtr_front = speed;

        right_mtr_back = speed;
        right_mtr_front = speed;


      } else if (lValue > rValue) { // the left side is going faster than the right and needs to be slowed down

        correctionValue = speed * callibrationSettings::TURN_CORRECTION;

        left_mtr_back = correctionValue;
        left_mtr_front = correctionValue;

        right_mtr_back = speed;
        right_mtr_front = speed;


      }else if (lValue < rValue) { // the right side is going faster than the left and needs to be slowed down

        correctionValue = speed * callibrationSettings::TURN_CORRECTION;


        left_mtr_back = speed;
        left_mtr_front = speed;

        right_mtr_back = correctionValue;
        right_mtr_front = correctionValue;

      }

    }

    //set all motors to zero after the bot has gone the specified amount
    left_mtr_back = 0;
    left_mtr_front = 0;

    right_mtr_back = 0;
    right_mtr_front = 0;

    return true;
  }
}

bool turn(float theta, int speed) { //theta is in degrees
    std::cout << "turn called for theta: " << theta << " and speed " << speed << "\n";

    double wheelDegrees = (callibrationSettings::wheelBaseRadius * theta) / callibrationSettings::wheelRadius;

    wheelDegrees *= .75; //this number was derived form the callibrator and is likely different for each robot

    speed = abs(speed);

  	int lMulti = 1;
  	int rMulti = -1;


    double lValue,lStart;
    double rValue,rStart;

    if ((lTarget == ERROR) || (rTarget == ERROR)) {
      lStart = left_mtr_back.get_position();
      rStart = right_mtr_back.get_position();
    }else{
      lStart = lTarget;
      rStart = rTarget;

      //increment the target values with this request
      lTarget += lMulti*wheelDegrees;
      rTarget += rMulti*wheelDegrees;
    }

    lValue = left_mtr_back.get_position();
    rValue = right_mtr_back.get_position();

      setDriveTrainPIDIsActivated(false);

      std::cout << "PID turning " << wheelDegrees << " degrees\n";
      std::cout << "lStart: " << lStart << "\n";
      std::cout << "rStart: " << rStart << "\n";

      setLeftDriveTrainTarget(lMulti*wheelDegrees + lStart,speed);
      setRightDriveTrainTarget(rMulti*wheelDegrees + rStart,speed);

      std::cout << "leftDriveTrainTarget: " << lMulti*wheelDegrees + lStart<< " degs \n";
      std::cout << "rightDriveTrainTarget: " << rMulti*wheelDegrees + rStart<< " degs \n";

      double lSpeed = speed, rSpeed = speed;
      int counter = 0;
      setDriveTrainPIDIsActivated(true);
      while ( ( fabs( fabs(lValue-lStart) - wheelDegrees) > callibrationSettings::MOTOR_POSITION_ERROR) || ( fabs( fabs(rValue-rStart) - wheelDegrees) > callibrationSettings::MOTOR_POSITION_ERROR)){
        lValue = left_mtr_back.get_position();
        rValue = right_mtr_back.get_position();

        if ((left_mtr_back.get_actual_velocity() == 0) && (right_mtr_back.get_actual_velocity() == 0) ) {
          speed+= 0.1;
        }

        double vDiff = fabs(left_mtr_back.get_actual_velocity()) - fabs(right_mtr_back.get_actual_velocity());


        lSpeed = speed + vDiff;
        rSpeed = speed - vDiff;

        setLeftDriveTrainTarget(lMulti*wheelDegrees + lStart, lSpeed);
        setRightDriveTrainTarget(rMulti*wheelDegrees + rStart,rSpeed);

/*
        //turn correction
        if ( fabs(fabs(lValue-lStart) - fabs(rValue-rStart)) < 0.5 * callibrationSettings::MOTOR_POSITION_ERROR) {
          double turn_error = fabs(lValue-lStart) - fabs(rValue-rStart);
          if (turn_error < 0) { //left is falling behind.. slow down right
            turn_error = fabs(turn_error);
            setLeftDriveTrainTarget(lMulti*wheelDegrees + lStart,speed + turn_error*0.5/callibrationSettings::TURN_CORRECTION);
            setRightDriveTrainTarget(rMulti*wheelDegrees + rStart, speed - turn_error*0.5/callibrationSettings::TURN_CORRECTION);
          }else{//right is falling behind.. slow down left
            turn_error = fabs(turn_error);
            setLeftDriveTrainTarget(lMulti*wheelDegrees + lStart,speed - turn_error*0.5/callibrationSettings::TURN_CORRECTION);
            setRightDriveTrainTarget(rMulti*wheelDegrees + rStart,speed + turn_error*0.5/callibrationSettings::TURN_CORRECTION);
          }
        }else{
          setLeftDriveTrainTarget(lMulti*wheelDegrees + lStart,speed);
          setRightDriveTrainTarget(rMulti*wheelDegrees + rStart,speed);
        }
*/

        if (counter %100==0) std::cout << "turning straight\nlValue: " << lValue << "\nrValue: " << rValue << "\n counter: " << counter << "--\n";
        pros::delay(1);
        autoPilotController(counter);
        counter++;
        checkForStop();

        if (counter > fabs(10 + speed * theta/M_PI) ) {
          consoleLogN("-ERROR- Turn taking too long. Terminating...");
          std::cout << "-ERROR- Turn of magn: " << theta << " degs and speed: " << speed << " taking too long. (counter = " << counter << ">" << fabs(speed * theta/M_PI) << ") \n" << "terminating turn... \n";\
          break;
        }

      }

      setDriveTrainPIDIsActivated(false);
      left_mtr_back .move(0);
      right_mtr_back .move(0);
      left_mtr_front .move(0);
      right_mtr_front .move(0);

      pros::motor_brake_mode_e_t prevBrakeMode = left_mtr_back.get_brake_mode();

      left_mtr_back.set_brake_mode(MOTOR_BRAKE_HOLD);
      right_mtr_back.set_brake_mode(MOTOR_BRAKE_HOLD);
      right_mtr_front.set_brake_mode(MOTOR_BRAKE_HOLD);
      left_mtr_front.set_brake_mode(MOTOR_BRAKE_HOLD);
      pros::delay(100);
      left_mtr_back.set_brake_mode(prevBrakeMode);
      right_mtr_back.set_brake_mode(prevBrakeMode);
      right_mtr_front.set_brake_mode(prevBrakeMode);
      left_mtr_front.set_brake_mode(prevBrakeMode);
      std::cout << "finalPos:\n" << lValue << "\n" << rValue << "\n--\n";

      std::vector<GEAH::Motor>motors{left_mtr_back,right_mtr_back,left_mtr_front,right_mtr_front};
      GEAH::autonRequestReceipt receipt("driveStraight",motors,theta,speed,MOVE_DEGREES);
      recordRobotAutonMovement(receipt); //<- used for odometry and auton callibration

      return true;
}

bool moveMotor(GEAH::Motor motor, float magnatude, int speed, int type) {
    //for type:
    //0 = rotations
    //1 = meters

    float give = 0.1f;

    float wheelRotations;

    if (type == MOVE_ROTATIONS) {
      wheelRotations = magnatude;
    }else if (type == MOVE_METERS) {
      wheelRotations = magnatude / ( 2 * callibrationSettings::wheelRadius * M_PI);
    }else if (type == MOVE_DEGREES){
      wheelRotations = magnatude / 360;
    }else{
      //default to distance
      wheelRotations = magnatude / ( 2 * callibrationSettings::wheelRadius * M_PI);
    }

    if (magnatude < 0) speed = -fabs(speed);

    magnatude = fabs(magnatude);


    double value,start;

    value = 0.000001;

    start = motor.get_position();


    float degs = fabs(360*wheelRotations);

    int motorCannotMoveCounter = 0;

    while (value < degs ) {
        checkForStop(); //this method should be continuously called the entire duration of the program

        value = fabs(motor.get_position()-start);

        motor = speed;

        if (fabs(motor.get_actual_velocity()) <= 0.01) motorCannotMoveCounter++;

        if (motorCannotMoveCounter > 2000) {
          consoleLogN("Error:: motor cannot move");
          consoleLogN("Movement of Motor Terminated");
          motor.move(0);
          return false; //motor movement was in some way inhibited
        }
    }

    //set all motors to zero after the bot has gone the specified amount
    motor.move(0);

    std::vector<GEAH::Motor>motors{motor};
    GEAH::autonRequestReceipt receipt("moveMotor",motors,magnatude,speed,type);
    recordRobotAutonMovement(receipt); //<- used for odometry and auton callibration

    return true;
}

bool setMotorPosition(GEAH::Motor motor, float position, int speed, int type) { //return true if successful, false if not
  //for type:
  //0 = rotations
  //1 = meters
  //2 = degrees

  float magnatude = position;

  float wheelRotations;

  float degs;

  if (type == MOVE_ROTATIONS) {
    wheelRotations = magnatude;
    degs = fabs(360*wheelRotations);
  }else if (type == MOVE_METERS) {
    wheelRotations = magnatude / ( 2 * callibrationSettings::wheelRadius * M_PI);
    degs = fabs(360*wheelRotations);
  }else if (type == MOVE_DEGREES) {
    degs = magnatude;
  }else{
    //default to distance
    wheelRotations = magnatude / ( 2 * callibrationSettings::wheelRadius * M_PI);
    degs = fabs(360*wheelRotations);
  }


  //if the motor is already at the correct position then don't move
//  if (fabs(motor.get_position() - degs) < callibrationSettings::MOTOR_POSITION_ERROR) return;

  double value,start;

  value = 0.000001;

  start = motor.get_position();

  pros::motor_brake_mode_e_t prevBrakeMode = motor.get_brake_mode();
  motor.set_brake_mode(MOTOR_BRAKE_HOLD);

  if (motor.get_position() < degs) {
    speed = fabs(speed);
  }else {
    speed = -fabs(speed);
  }

  float min_V = 0.1f; //the minimum voltage the motor will use

  while ( fabs(value) < fabs(degs-start)) {
      checkForStop(); //this method should be continuously called the entire duration of the program

      value = fabs(motor.get_position()-start);

      motor.move_velocity(min_V*speed/fabs(speed) + fabs(value - (degs-start))*speed/fabs(127)/180);

      if (fabs(motor.get_actual_velocity()) <= 0.03) {
        min_V+= 0.005; //if the motor is stuck then up the voltage

        if (fabs(min_V*speed/fabs(speed) + fabs(value - (degs-start))*speed/fabs(127)/180) > 220) {
          consoleLogN("Error:: motor cannot move");
          consoleLogN("Movement of Motor Terminated");
          motor.move(0);
          return false; //motor movement was in some way inhibited
        }
      }
  }

  //reverse motor to counter momentum
  motor .move(0);
  pros::delay(150);
  motor.set_brake_mode(prevBrakeMode);

  std::vector<GEAH::Motor>motors{motor};
  GEAH::autonRequestReceipt receipt("setMotorPosition",motors,magnatude,speed,type);
  recordRobotAutonMovement(receipt); //<- used for odometry and auton callibration

  return true; // movement was successful
}

constexpr int SIDE_LEFT = 1;
constexpr int SIDE_RIGHT = -1;

void runAutoPilot(int times) {
  int count = 0;
  while (count < times) {
    autoPilotController(count);
    count++;
    pros::delay(10);
  }
}



void moveSquares(double numSquares) {
  driveStraight(0.6 * numSquares,127,MOVE_METERS);
}

void bumpWall() {
  bool prevPidSetting = usePIDForDriveTrainAutonMovement;
  usePIDForDriveTrainAutonMovement = false;
  driveStraight(-0.12, 127,MOVE_METERS);
  pros::delay(200);
  driveStraight(0.12, 127,MOVE_METERS);
  usePIDForDriveTrainAutonMovement = prevPidSetting;
}

void extendRamp() {
  //assume starting with intake_lift at 90 degree position (straight up)

  pros::motor_brake_mode_e_t prevBrakeMode = intake_lift_mtr.get_brake_mode();

  moveMotor(intake_lift_mtr,-30 * 84/12,255,MOVE_DEGREES);
  intake_lift_mtr.set_brake_mode(MOTOR_BRAKE_HOLD);
  moveMotor(ramp_mtr,45,100,MOVE_DEGREES);
  pros::delay(100);
  moveMotor(ramp_mtr,-45,100,MOVE_DEGREES);
  intake_lift_mtr.set_brake_mode(prevBrakeMode);
  pros::delay(100);
}


void autonomous(int auton_sel);

void autonomous() {
  autonomous(getAuton());
}

void autonomous(int auton_sel) {
  setDriveTrainPIDIsActivated(true);

  lTarget = lTarget == ERROR ? left_mtr_back.get_position() : lTarget;
  rTarget = rTarget == ERROR ? right_mtr_back.get_position() : rTarget;

  extern bool useFrontSensorForDistance;

  switch(auton_sel) {

    case(0): //near left
    extendRamp();
    left_intake.move(255);
    right_intake.move(255);
    moveSquares(1);
    left_intake.move(0);
    right_intake.move(0);
    turn(90,255);

    bumpWall();

    break;

    case(1): //near right
    extendRamp();
    left_intake.move(255);
    right_intake.move(255);
    moveSquares(2);
    left_intake.move(0);
    right_intake.move(0);
    break;

    case(2): //left side far
    //hello good morning how are you doing is this working my
    //shift isn't gosh blehhh imma hold down my shft keys until they do now lol
    //hmmmmmmmm
    //system.out.println9'
    break;


    case(3): //right side far

    break;

    case(4): //far-noPark

      break;

    case(5): //experimental left far

      break;

    case(6): //experimental far right

    break;

    case(7): //skills

    break;

    default:
      LEDs(true);
      pros::delay(100);
      LEDs(false);
      pros::delay(200);
      LEDs(true);
      pros::delay(100);
      LEDs(false);
    break;
  }
  setDriveTrainPIDIsActivated(false);

  std::cout << "-END OF AUTON:  " << getAutonName(auton_sel) << "-\n";
  std::cout << "lTarget: " << lTarget << "\n";
  std::cout << "lTarget: " << lTarget << "\n";

  lTarget = ERROR;
  rTarget = ERROR;
}
