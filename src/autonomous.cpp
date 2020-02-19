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


bool autonLockWheelsIntake = false;

double lTarget = ERROR, rTarget = ERROR;
int aTLoops = 0;
bool checkForStop() {
  return false;
}

bool usePIDForDriveTrainAutonMovement = true;

long autoPilotLoops = 0;
void concurrentOperations() {
  if (autonLockWheelsIntake) {
    pros::delay(1);
    left_intake.move(1.1 * left_mtr_front.get_actual_velocity());
    right_intake.move(1.1 * left_mtr_front.get_actual_velocity());
    pros::delay(1);
  }

  autoPilotController(autoPilotLoops);
  autoPilotLoops++;
}
/*
bool driveStraight(double magnatude, double speed, int type) {
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


      std::cout << "driveTrainTarget: " << wheelDegrees + lStart << " degs \n";

      int counter = 0;

      setDriveTrainPIDIsActivated(true);

      int motorCannotMoveCounter = 0;
      double lSpeed = speed, rSpeed = speed;
      double driveSpeed = speed;



std::cout << "setting drivetrain target" << std::endl;
pros::delay(10);
      //WE ONLY CALL THESE ONCE NOW
      setLeftDriveTrainTarget(wheelDegrees + lStart, speed);
      setRightDriveTrainTarget(wheelDegrees + rStart,speed);

std::cout << "begin auton loop" << std::endl;
pros::delay(10);
      while ( (fabs( lValue - (wheelDegrees+lStart)) >= callibrationSettings::MOTOR_POSITION_ERROR) || ( fabs( rValue - (wheelDegrees+rStart)) >= callibrationSettings::MOTOR_POSITION_ERROR)){
      //  if (driveSpeed < speed) driveSpeed += speed/3;

        lValue = left_mtr_back.get_position();
        rValue = right_mtr_back.get_position();


        double vDiff = fabs(left_mtr_back.get_actual_velocity()) - fabs(right_mtr_back.get_actual_velocity());
        //double vDiff = fabs(lValue-(wheelDegrees+lStart)) - fabs(rValue-(wheelDegrees+rStart));

        if (speed >= 0) {
          lSpeed = driveSpeed + vDiff*callibrationSettings::TURN_CORRECTION;
          rSpeed = driveSpeed - vDiff*callibrationSettings::TURN_CORRECTION;
        }else{
          lSpeed = driveSpeed - vDiff*callibrationSettings::TURN_CORRECTION;
          rSpeed = driveSpeed + vDiff*callibrationSettings::TURN_CORRECTION;
        }

        setLeftDriveTrainPIDSpeedModifier(lSpeed);
        setRightDriveTrainPIDSpeedModifier(rSpeed);



        if (counter %500==0) std::cout << "driving straight\nlValue: " << lValue << "\nrValue: " << rValue << "\ncounter:" << counter << "\n--\n";

        autoPilotController(counter);
        counter++;
        checkForStop();
        concurrentOperations();

        if (counter > fabs(10 + 10*(wheelDegrees/M_PI)/speed) ) {
          consoleLogN("-ERROR- Movement taking too long. Terminating...");
          std::cout << "-ERROR- DriveStraight of magn: " << wheelDegrees << "degs and speed: " << speed << " taking too long. (counter = " << counter << ") \n" << "terminating turn... \n";
          break;
        }



        if (fabs(left_mtr_back.get_actual_velocity()) < 0.05 * driveSpeed ) {
            motorCannotMoveCounter++;
            driveSpeed *= 1.05;
        }else{
            motorCannotMoveCounter/=2;
        }

        if (motorCannotMoveCounter > 1000) {
          consoleLogN("Error:: drivetrain cannot move");
          consoleLogN("DriveStraight Terminated");
          std::cout << "Drivetrain cannot move, terminating turn " << "\n ActualVelocity:" << left_mtr_back.get_actual_velocity() << "\n";
          setDriveTrainPIDIsActivated(false);
          left_mtr_back.move(0);
          right_mtr_back.move(0);
          left_mtr_front.move(0);
          right_mtr_front.move(0);
          return false; //motor movement was in some way inhibited
        }

        pros::delay(1);

      }

      setDriveTrainPIDIsActivated(false);
      left_mtr_back.move(0);
      right_mtr_back.move(0);
      left_mtr_front.move(0);
      right_mtr_front.move(0);

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


      std::cout << "finalPos:" << std::endl << lValue << std::endl << rValue << std::endl;

      GEAH::Motor motorsArr[] = {left_mtr_back,right_mtr_back,left_mtr_front,right_mtr_front};
      std::cout << "recording movement..." << std::endl;
      pros::delay(10);
//      std::vector<GEAH::Motor> motors (motorsArr, motorsArr + sizeof(motorsArr));
      std::cout << "recording movement..." << std::endl;
      pros::delay(10);
  //    GEAH::autonRequestReceipt receipt("driveStraight",motors,magnatude,speed,type);


//      recordRobotAutonMovement(receipt); //<- used for odometry and auton callibration
      return true;

    }else{

    float degs = fabs(360*wheelRotations);

    int motorCannotMoveCounter = 0;

    while (lValue < degs ) {


      checkForStop(); //this method should be continuously called the entire duration of the program
      concurrentOperations();

      lValue = fabs(left_mtr_back.get_position()-lStart);
      rValue = fabs(right_mtr_back.get_position()-rStart);

      if (fabs(left_mtr_back.get_actual_velocity()) <= 0.1) motorCannotMoveCounter++;

      if (motorCannotMoveCounter > 700) {
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
*/

/*
bool turn(double theta, int speed) { //theta is in degrees
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


      std::cout << "leftDriveTrainTarget: " << lMulti*wheelDegrees + lStart<< " degs \n";
      std::cout << "rightDriveTrainTarget: " << rMulti*wheelDegrees + rStart<< " degs \n";

      int motorCannotMoveCounter = 0;

      double lSpeed = speed, rSpeed = speed;
      int counter = 0;
      double driveSpeed = speed;
      setDriveTrainPIDIsActivated(true);

      left_mtr_back.set_brake_mode(MOTOR_BRAKE_BRAKE);
      right_mtr_back.set_brake_mode(MOTOR_BRAKE_BRAKE);
      right_mtr_front.set_brake_mode(MOTOR_BRAKE_BRAKE);
      left_mtr_front.set_brake_mode(MOTOR_BRAKE_BRAKE);

      //THESE ARE ONLY CALLED ONCE NOW
      setLeftDriveTrainTarget(lMulti*wheelDegrees + lStart, lSpeed);
      setRightDriveTrainTarget(rMulti*wheelDegrees + rStart,rSpeed);

      while ( ( fabs( fabs(lValue-lStart) - wheelDegrees) >= callibrationSettings::MOTOR_POSITION_ERROR) || ( fabs( fabs(rValue-rStart) - wheelDegrees) >= callibrationSettings::MOTOR_POSITION_ERROR)){
        //if (driveSpeed < speed) driveSpeed += speed/3; //gradually increase the speed of the motors

        lValue = left_mtr_back.get_position();
        rValue = right_mtr_back.get_position();

        if ((left_mtr_back.get_actual_velocity() == 0) && (right_mtr_back.get_actual_velocity() == 0) ) {
          driveSpeed+= 0.1;
          motorCannotMoveCounter++;
        }

        double vDiff = fabs(left_mtr_back.get_actual_velocity()) - fabs(right_mtr_back.get_actual_velocity());
        //double vDiff = fabs(fabs( fabs(lValue-lStart) - wheelDegrees) - fabs( fabs(rValue-rStart) - wheelDegrees));

        if (lSpeed >= 0) {
          lSpeed = driveSpeed + vDiff*callibrationSettings::TURN_CORRECTION;
        }else{
          lSpeed = driveSpeed - vDiff*callibrationSettings::TURN_CORRECTION;
        }

        if (rSpeed >= 0) {
          rSpeed = driveSpeed - vDiff*callibrationSettings::TURN_CORRECTION;
        }else{
          rSpeed = driveSpeed + vDiff*callibrationSettings::TURN_CORRECTION;
        }

        setLeftDriveTrainPIDSpeedModifier(lSpeed);
        setRightDriveTrainPIDSpeedModifier(rSpeed);

        autoPilotController(counter);

        if (counter %100==0) std::cout << "turning straight\nlValue: " << lValue << "\nrValue: " << rValue << "\n counter: " << counter << "--\n";

        counter++;
        checkForStop();
        concurrentOperations();

        if (counter > fabs(20 + 50*(theta/M_PI)/driveSpeed) ) {
          consoleLogN("-ERROR- Turn taking too long. Terminating...");
          std::cout << "-ERROR- Turn of magn: " << theta << " degs and speed: " << speed << " taking too long. (counter = " << counter << ">" << fabs(20 + 300*(theta/M_PI)/speed) << ") \n" << "terminating turn... \n";\
          break;
        }

        if (motorCannotMoveCounter > 1000) {
          consoleLogN("Error:: drivetrain cannot move");
          consoleLogN("Turn Terminated");
          std::cout << "Drivetrain cannot move, terminating turn " << "\n ActualVelocity:" << left_mtr_back.get_actual_velocity() << "\n";
          setDriveTrainPIDIsActivated(false);
          left_mtr_back.move(0);
          right_mtr_back.move(0);
          left_mtr_front.move(0);
          right_mtr_front.move(0);
          return false; //motor movement was in some way inhibited
        }

        pros::delay(1);

      }

      setDriveTrainPIDIsActivated(false);
      left_mtr_back.move(0);
      right_mtr_back.move(0);
      left_mtr_front.move(0);
      right_mtr_front.move(0);

      pros::motor_brake_mode_e_t prevBrakeMode = left_mtr_back.get_brake_mode();


      pros::delay(100);
      left_mtr_back.set_brake_mode(prevBrakeMode);
      right_mtr_back.set_brake_mode(prevBrakeMode);
      right_mtr_front.set_brake_mode(prevBrakeMode);
      left_mtr_front.set_brake_mode(prevBrakeMode);
      std::cout << "finalPos:\n" << lValue << "\n" << rValue << "\n--\n";

      GEAH::Motor motorsArr[] = {left_mtr_back,right_mtr_back,left_mtr_front,right_mtr_front};
      std::vector<GEAH::Motor> motors (motorsArr, motorsArr + sizeof(motorsArr));

      GEAH::autonRequestReceipt receipt("driveStraight",motors,theta,speed,MOVE_DEGREES);
      recordRobotAutonMovement(receipt); //<- used for odometry and auton callibration

      return true;
}
*/



bool driveTrainPIDControlFunction(double magnatude,double theta, double setSpeed) {
  double degs = 360 * magnatude / ( 2 * callibrationSettings::wheelRadius* M_PI);

  if (degs > 0) {
    consoleLog("driving ");
    consoleLog(degs);
    consoleLogN(" degrees.");
  }

  double turnDegs = (callibrationSettings::wheelBaseRadius * theta) / callibrationSettings::wheelRadius;
  turnDegs *= .75; //this number was derived form the callibrator and is likely different for each robot

  if (turnDegs > 0) {
    consoleLog("driving ");
    consoleLog(turnDegs);
    consoleLogN(" degrees.");
  }

  double lValue = left_mtr_back.get_position(), rValue = right_mtr_back.get_position();
  lTarget += degs + turnDegs;
  rTarget += degs - turnDegs;

  double lStart = lValue, rStart = rValue;
  //reset the pids incase we've used them before
  left_mtr_back.resetPID();
  right_mtr_back.resetPID();

  left_mtr_back.setAPIDTarget(lTarget);
  right_mtr_back.setAPIDTarget(rTarget);

  double speed = 1, lSpeed = 0, rSpeed = 0;

  left_mtr_back.setSpeedModifier(&lSpeed);
  right_mtr_back.setSpeedModifier(&rSpeed);

  int cannotMoveCounter = 0;
  while ( (std::abs(lTarget-lValue) > callibrationSettings::MOTOR_POSITION_ERROR) || (std::abs(rTarget-rValue) > callibrationSettings::MOTOR_POSITION_ERROR) ) {
    if (speed < setSpeed) speed+=2;

    concurrentOperations();

    left_mtr_back.runPid();
    right_mtr_back.runPid();
    left_mtr_front.move_voltage(left_mtr_back.get_voltage());
    right_mtr_front.move_voltage(left_mtr_back.get_voltage());

    lValue = left_mtr_back.get_position();
    rValue = right_mtr_back.get_position();

    //drive correction
    //double vDiff = std::abs(left_mtr_back.get_actual_velocity()) - std::abs(right_mtr_back.get_actual_velocity());
    double vDiff = std::abs((lTarget - lValue)/(lTarget-lStart)) - std::abs((rTarget - rValue)/(lTarget-lStart));

    lSpeed = speed + vDiff*callibrationSettings::TURN_CORRECTION;
    rSpeed = speed - vDiff*callibrationSettings::TURN_CORRECTION;

    pros::delay(2);

    if ((std::abs(left_mtr_back.get_actual_velocity()) < 0.1 * ((double) std::abs(left_mtr_back.get_target_velocity()))) && (std::abs(right_mtr_back.get_actual_velocity()) < 0.1 * ((double) std::abs(right_mtr_back.get_target_velocity())))  ) {
      cannotMoveCounter++;
    }else{
      cannotMoveCounter = 0;
    }

    if (cannotMoveCounter > 200) {
      consoleLogN("drivetrain cannot move. terminating motion");
      std::cout << "drivetrain cannot move. Terminating motion..." << std::endl;
      lValue = lTarget;
      rValue = rTarget;
      return false;
    }

  }

  left_mtr_back.move(0);
  right_mtr_back.move(0);
  left_mtr_front.move(0);
  right_mtr_front.move(0);

  return true;
}

bool drive(double magnatude, double speed) {
  left_mtr_back.setKM(callibrationSettings::DrivetrainKM);
  right_mtr_back.setKM(callibrationSettings::DrivetrainKM);
  return driveTrainPIDControlFunction(magnatude,0,speed);
}

bool turn(double magnatude, double speed) {
  left_mtr_back.setKM(1 * callibrationSettings::DrivetrainKM);
  right_mtr_back.setKM(1 * callibrationSettings::DrivetrainKM);
  return driveTrainPIDControlFunction(0,magnatude,100 * speed);
}

bool moveMotor(GEAH::Motor motor, float magnatude, int speed, int type) {
    //for type:
    //0 = rotations
    //1 = meters

    float give = 0.1f;

    float wheelRotations;

    std::cout << "moving " << motor.getName() << "\n";

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
        concurrentOperations();

        value = fabs(motor.get_position()-start);

        motor.move(speed);

        if (fabs(motor.get_actual_velocity()) < 0.05 * speed ) {
          motorCannotMoveCounter++;
        }else{
            motorCannotMoveCounter/=2;
        }

        if (motorCannotMoveCounter > 1000) {
          consoleLogN("Error:: motor cannot move");
          consoleLogN("Movement of Motor Terminated");
          std::cout << "Motor cannot move, terminating motor " << motor.getName() << "\n Speed:" << motor.get_actual_velocity() << "\n";
          motor.move(0);
          return false; //motor movement was in some way inhibited
        }

        pros::delay(1);
    }

    //set motor to zero after the bot has gone the specified amount
    motor.move(0);


    GEAH::Motor motorsArr[] = {motor};
    std::vector<GEAH::Motor> motors (motorsArr, motorsArr + sizeof(motorsArr));

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

  float min_V = 0.5f; //the minimum voltage the motor will use

  motor.setAPIDTarget(value);

  while ( fabs(value) < fabs(degs-start)) {
      checkForStop(); //this method should be continuously called the entire duration of the program
      concurrentOperations();

      value = fabs(motor.get_position()-start);

      motor.move_velocity(min_V*speed/fabs(speed) + 10*fabs(value - (degs-start))*speed/fabs(127)/180);

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
  motor.move(0);
  pros::delay(150);
  motor.set_brake_mode(prevBrakeMode);

  GEAH::Motor motorsArr[] = {motor};
  std::vector<GEAH::Motor> motors (motorsArr, motorsArr + sizeof(motorsArr));
  GEAH::autonRequestReceipt receipt("setMotorPosition",motors,magnatude,speed,type);
  recordRobotAutonMovement(receipt); //<- used for odometry and auton callibration

  return true; // movement was successful
}


bool setAPIDPosition(GEAH::Motor motor, double degs, double speed) { //return true if successful, false if not
  motor.resetPID();
  motor.setAPIDTarget(degs);

  double value = motor.get_position();

  motor.setSpeedModifier(&speed);

  int cannotMoveCounter = 0;
  while ( std::abs(value-degs) > callibrationSettings::MOTOR_POSITION_ERROR) {
    concurrentOperations();

    motor.runPid();

    value = motor.get_position();

    pros::delay(2);

    if ((std::abs(motor.get_actual_velocity()) < 0.1 * ((double) std::abs(motor.get_target_velocity()))) ) {
      cannotMoveCounter++;
    }else{
      cannotMoveCounter = 0;
    }

    if (cannotMoveCounter > 100) {
      consoleLogN("drivetrain cannot move. terminating motion");
      std::cout << "drivetrain cannot move. Terminating motion..." << std::endl;
      return false;
    }

  }

  motor = 0;


  return true; // movement was successful
}

constexpr int SIDE_LEFT = 1;
constexpr int SIDE_RIGHT = -1;

void runAutoPilot(int times) {
  int count = 0;
  while (count < times) {
    autoPilotController(count);
    concurrentOperations();
    count++;
    pros::delay(10);
  }
}



void moveSquares(double numSquares, double speed) {
  drive(0.6 * numSquares,speed);
}
void moveSquares(double numSquares) {
  moveSquares(numSquares,100);
}

void bumpWall() {
  bool prevPidSetting = usePIDForDriveTrainAutonMovement;
  usePIDForDriveTrainAutonMovement = false;
  drive(-0.12, 127);
  pros::delay(200);
  drive(0.12,127);
  usePIDForDriveTrainAutonMovement = prevPidSetting;
}


void extendRampAndMoveSquares(double squares) { //Alec's ramp extending method
    left_intake.move(-255);
    right_intake.move(-255);
    moveSquares(squares);
    left_intake.move(0);
    right_intake.move(0);
    std::cout << "extended ramp" << std::endl;
    pros::delay(10);
}

void stack(int blockNum) { //Alec's stacking method
  setDriveTrainPIDIsActivated(false);
  lTarget = left_mtr_back.get_position();
  rTarget = right_mtr_back.get_position();
  if (blockNum > 4) {
    left_intake.move(10 * blockNum-1);
    right_intake.move(10 * blockNum-1);
  }else{
    left_intake.move(10);
    right_intake.move(10);
  }

    double k = 2 * 10/blockNum;
    double targ = 87*84/6 + blockNum-10;
    double pos = ramp_mtr.get_position();
    while (pos < targ) {
       pos = ramp_mtr.get_position();
       if ((targ-pos) > .65 * targ) {
         ramp_mtr.move_velocity(k * (targ-pos));
       }else{
         ramp_mtr.move_velocity(10 + (targ-pos)/(blockNum));
       }

    }
    ramp_mtr.move(0);


    left_intake.move(0);
    right_intake.move(0);
    pros::delay(500);
    autonLockWheelsIntake = true;
    moveSquares(-0.7);
    autonLockWheelsIntake = false;
    left_intake.move(0);
    right_intake.move(0);
}

void resetAutonTargets() {
  lTarget = left_mtr_back.get_position();
  rTarget = right_mtr_back.get_position();
}

void autonomous(int auton_sel);

void autonomous() {
  left_mtr_back.set_brake_mode(MOTOR_BRAKE_BRAKE);
  right_mtr_back.set_brake_mode(MOTOR_BRAKE_BRAKE);
  left_mtr_front.set_brake_mode(MOTOR_BRAKE_BRAKE);
  right_mtr_front.set_brake_mode(MOTOR_BRAKE_BRAKE);
  autonomous(getAuton(),1);
  left_mtr_back.set_brake_mode(MOTOR_BRAKE_COAST);
  right_mtr_back.set_brake_mode(MOTOR_BRAKE_COAST);
  left_mtr_front.set_brake_mode(MOTOR_BRAKE_COAST);
  right_mtr_front.set_brake_mode(MOTOR_BRAKE_COAST);
}

void delay(int time) {
  pros::delay(time);
}

  void grabAndStackAuton(int side, int mode) {
    if (mode == 1) {
  	      extendRampAndMoveSquares(-0.3);
  	    }else{
  	      moveSquares(-0.3);
  	    }

  	    turn(-90 * side,100);
  	    bumpWall();
  	    left_intake.move(255);
  	    right_intake.move(255);
  	    moveSquares(1);
  	    pros::delay(5);
  	    moveSquares(0.5);
  	    left_intake.move(0);
  	    right_intake.move(0);
  	    moveSquares(-1.5);
  	    turn(-90 * side,100);
  	    moveSquares(0.8);
  	    stack(5);
  }

void autonomous(int auton_sel,int mode) {
  setDriveTrainPIDIsActivated(true);

  lTarget = lTarget == ERROR ? left_mtr_back.get_position() : lTarget;
  rTarget = rTarget == ERROR ? right_mtr_back.get_position() : rTarget;

  extern bool useFrontSensorForDistance;

  switch(auton_sel) {
	    case(0)://forward-up
	     if (mode ==  1) {
	       extendRampAndMoveSquares(1.2);
	     }else{
	       moveSquares(1.2);
	     }
	     delay(500);
	    moveSquares(-1.2);
	   break;

	   case(1): //blue left
	   extendRampAndMoveSquares(0.3);

	   left_intake.move(255);
	   right_intake.move(255);

	   //pick up cubes
	   moveSquares(1.6,40);

	   left_intake.move(0);
	   right_intake.move(0);

	   moveSquares(-1.25,125);
	   turn(-130,150);

	   left_intake.move(125);
	   right_intake.move(125);
	   moveSquares(0.66,125);
	   left_intake.move(0);
	   right_intake.move(0);

	   stack(4);


	   break;

	   case(2): //blue right

	   extendRampAndMoveSquares(0.3);
	   left_intake.move(255);
	   right_intake.move(255);
	   moveSquares(1.1);
	   delay(500);
	   moveSquares(0.6);
	   delay(5);
	   left_intake.move(0);
	   right_intake.move(0);
	   moveSquares(-1.85);
	   turn(90,100);
	   moveSquares(1.21);
	   stack(5);


	   break;


	   case(3): //red left
	   extendRampAndMoveSquares(0.3);
	   left_intake.move(255);
	   right_intake.move(255);
	   moveSquares(1.1);
	   delay(500);
	   moveSquares(0.6);
	   delay(5);
	   left_intake.move(0);
	   right_intake.move(0);
	   moveSquares(-1.85);
	   turn(-90,100);
	   moveSquares(0.7);
	   stack(4);
	   break;

	   case(4): //red right
	   extendRampAndMoveSquares(0.3);

	   left_intake.move(255);
	   right_intake.move(255);

	   //pick up cubes
	   moveSquares(1.6,40);

	   left_intake.move(0);
	   right_intake.move(0);

	   moveSquares(-1.25,125);
	   turn(130,150);

	   left_intake.move(125);
	   right_intake.move(125);
	   moveSquares(0.66,125);
	   left_intake.move(0);
	   right_intake.move(0);

	   stack(4);
	   break;

	   case(5): //blue left 8 stak
	   extendRampAndMoveSquares(2.35);

	   delay(5);
	   turn(90 * SIDE_LEFT,255);
	   moveSquares(1);
	   left_intake.move(0);
	   right_intake.move(0);
	   turn(90 * SIDE_LEFT,100);
	   left_intake.move(255);
	   right_intake.move(255);
	   moveSquares(2.3);
	   turn(90 * SIDE_LEFT,255);

	   moveSquares(1.7);

	   stack(8);
	   break;

	   case(6): //red right 8 stak
	     extendRampAndMoveSquares(2.35);

	   delay(5);
	   turn(90 * SIDE_RIGHT,255);
	   moveSquares(1);
	   left_intake.move(0);
	   right_intake.move(0);
	   turn(90 * SIDE_RIGHT,100);
	   left_intake.move(255);
	   right_intake.move(255);
	   moveSquares(2.3);
	   turn(90 * SIDE_RIGHT,255);

	   moveSquares(1.7);

	   stack(8);
	   break;

	   case(7): //stack
	     stack(10);
	     break;



	   case(8): //skills
	   extendRampAndMoveSquares(0.3);

	   left_intake.move(255);
	   right_intake.move(255);

	   //pick up cubes
	   moveSquares(1.6,40);

	   moveSquares(1);

	   moveSquares(1.7,40);

	   turn(-45, 155);

	   delay(50);
	   left_intake.move(50);
	   right_intake.move(50);

	   delay(100);
	   moveSquares(1);
	   left_intake.move(0);
	   right_intake.move(0);
	   stack(8);
	   //8 stack stacked

	   turn(135,155);

	   moveSquares(1.2);

	   left_intake.move(155);
	   right_intake.move(155);

	   moveSquares(0.4);

	   left_intake.move(0);
	   right_intake.move(0);

	   moveSquares(-0.2);

	   setAPIDPosition(ramp_mtr,60*84/12,155);
	   setAPIDPosition(intake_lift_mtr,90 * 84/12,155);


       left_intake.move(-125);
       right_intake.move(-125);

       delay(500);

       moveSquares(-0.5);

       left_intake.move(0);
       right_intake.move(0);

       setAPIDPosition(intake_lift_mtr,0,155);
       setAPIDPosition(ramp_mtr,0,155);
       //tower gotten

       moveSquares(-0.36);

       turn(90,155);

       moveSquares(1);

       left_intake.move(255);
       right_intake.move(255);
       moveSquares(0.5);
       left_intake.move(0);
       right_intake.move(0);

       moveSquares(-0.2);

       //lift arms
	     setAPIDPosition(ramp_mtr,60*84/12,155);
       setAPIDPosition(intake_lift_mtr,90*84/12,155);

       left_intake.move(-100);
       right_intake.move(-100);

       moveSquares(-0.5);

       left_intake.move(0);
       right_intake.move(0);

       setAPIDPosition(intake_lift_mtr,0,155);
       setAPIDPosition(ramp_mtr,0,155);
       //tower gotten

       moveSquares(-1);

       turn(-90,155);

       moveSquares(0.6);

       turn(90,155);


       left_intake.move(200);
       right_intake.move(200);
       moveSquares(4,40);
       left_intake.move(50);
       right_intake.move(50);

       moveSquares(0.5);

       turn(-90,155);

       moveSquares(3.2);

       stack(7);
       //stack

       moveSquares(-0.5);

       turn(-90,155);

       left_intake.move(200);
       right_intake.move(200);
       moveSquares(0.35);
       left_intake.move(0);
       right_intake.move(0);
       turn(-90,155);

       moveSquares(0.6);

       moveSquares(-0.2);

       //lift arms
	     setAPIDPosition(ramp_mtr,60*84/12,155);
       setAPIDPosition(intake_lift_mtr,90*84/12,155);

       left_intake.move(-100);
       right_intake.move(-100);

       moveSquares(-0.5);

       left_intake.move(0);
       right_intake.move(0);
       //tower gotten


	   break;

	   case(9): //calibration

	   break;

	   case(10): //none


	   break;
	 }
  setDriveTrainPIDIsActivated(false);

  std::cout << "-END OF AUTON:  " << getAutonName(auton_sel) << "-\n";
  std::cout << "lTarget: " << lTarget << "\n";
  std::cout << "lTarget: " << lTarget << "\n";

  lTarget = ERROR;
  rTarget = ERROR;
}
