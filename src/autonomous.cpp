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

      int motorCannotMoveCounter = 0;
      double lSpeed = speed, rSpeed = speed;
      while ( (fabs( lValue - (wheelDegrees+lStart)) > callibrationSettings::MOTOR_POSITION_ERROR) || ( fabs( rValue - (wheelDegrees+rStart)) > callibrationSettings::MOTOR_POSITION_ERROR)){
        lValue = left_mtr_back.get_position();
        rValue = right_mtr_back.get_position();


        double vDiff = fabs(left_mtr_back.get_actual_velocity()) - fabs(right_mtr_back.get_actual_velocity());

        lSpeed = speed + vDiff/2;
        rSpeed = speed - vDiff/2;


        setLeftDriveTrainTarget(wheelDegrees + lStart, lSpeed);
        setRightDriveTrainTarget(wheelDegrees + rStart,rSpeed);



        if (counter %500==0) std::cout << "driving straight\nlValue: " << lValue << "\nrValue: " << rValue << "\ncounter:" << counter << "\n--\n";
        pros::delay(5);
        autoPilotController(counter);
        counter++;
        checkForStop();
        concurrentOperations();

        if (counter > fabs(10 + 5*(wheelDegrees/M_PI)/speed) ) {
          consoleLogN("-ERROR- Turn taking too long. Terminating...");
          std::cout << "-ERROR- DriveStraight of magn: " << wheelDegrees << "degs and speed: " << speed << " taking too long. (counter = " << counter << ") \n" << "terminating turn... \n";
          break;
        }



        if (fabs(left_mtr_back.get_actual_velocity()) < 0.05 * speed ) {
            motorCannotMoveCounter++;
            speed *= 1.05;
        }else{
            motorCannotMoveCounter/=2;
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

      setLeftDriveTrainTarget(lMulti*wheelDegrees + lStart,speed);
      setRightDriveTrainTarget(rMulti*wheelDegrees + rStart,speed);

      std::cout << "leftDriveTrainTarget: " << lMulti*wheelDegrees + lStart<< " degs \n";
      std::cout << "rightDriveTrainTarget: " << rMulti*wheelDegrees + rStart<< " degs \n";

      int motorCannotMoveCounter = 0;

      double lSpeed = speed, rSpeed = speed;
      int counter = 0;
      setDriveTrainPIDIsActivated(true);

      left_mtr_back.set_brake_mode(MOTOR_BRAKE_BRAKE);
      right_mtr_back.set_brake_mode(MOTOR_BRAKE_BRAKE);
      right_mtr_front.set_brake_mode(MOTOR_BRAKE_BRAKE);
      left_mtr_front.set_brake_mode(MOTOR_BRAKE_BRAKE);

      while ( ( fabs( fabs(lValue-lStart) - wheelDegrees) > callibrationSettings::MOTOR_POSITION_ERROR) || ( fabs( fabs(rValue-rStart) - wheelDegrees) > callibrationSettings::MOTOR_POSITION_ERROR)){
        lValue = left_mtr_back.get_position();
        rValue = right_mtr_back.get_position();

        if ((left_mtr_back.get_actual_velocity() == 0) && (right_mtr_back.get_actual_velocity() == 0) ) {
          speed+= 0.1;
        }

        double vDiff = fabs(left_mtr_back.get_actual_velocity()) - fabs(right_mtr_back.get_actual_velocity());


        lSpeed = speed + vDiff*0.5;
        rSpeed = speed - vDiff*0.5;

        setLeftDriveTrainTarget(lMulti*wheelDegrees + lStart, lSpeed);
        setRightDriveTrainTarget(rMulti*wheelDegrees + rStart,rSpeed);

        if (counter %100==0) std::cout << "turning straight\nlValue: " << lValue << "\nrValue: " << rValue << "\n counter: " << counter << "--\n";
        pros::delay(1);
        autoPilotController(counter);
        counter++;
        checkForStop();
        concurrentOperations();

        if (counter > fabs(20 + 300*(theta/M_PI)/speed) ) {
          consoleLogN("-ERROR- Turn taking too long. Terminating...");
          std::cout << "-ERROR- Turn of magn: " << theta << " degs and speed: " << speed << " taking too long. (counter = " << counter << ">" << fabs(20 + 300*(theta/M_PI)/speed) << ") \n" << "terminating turn... \n";\
          break;
        }



        if (fabs(left_mtr_back.get_actual_velocity()) < 0.05 * speed ) {
          motorCannotMoveCounter++;
          speed *= 1.05;
        }else{
            motorCannotMoveCounter/=2;
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
    concurrentOperations();
    count++;
    pros::delay(10);
  }
}




void moveSquares(double numSquares) {
  driveStraight(0.6 * numSquares,200,MOVE_METERS);
}

void bumpWall() {
  bool prevPidSetting = usePIDForDriveTrainAutonMovement;
  usePIDForDriveTrainAutonMovement = false;
  driveStraight(-0.12, 127,MOVE_METERS);
  pros::delay(200);
  driveStraight(0.12, 127,MOVE_METERS);
  usePIDForDriveTrainAutonMovement = prevPidSetting;
}


void extendRampAndMoveSquares(double squares) { //Alec's ramp extending method

    setAPIDIsActivated("ramp_PID", true);
    setAPIDTargetAndSpeed("ramp_PID",  40 * 84/12, 255);
    left_intake.move(-255);
    right_intake.move(-255);
    setAPIDIsActivated("intake_lift_PID", true);
    pros::delay(250);
    setAPIDTargetAndSpeed("intake_lift_PID",  110 * 84/12, 255);
    pros::delay(150);
    moveSquares(squares);
    runAutoPilot(100);
    left_intake.move(0);
    right_intake.move(0);
    runAutoPilot(200);
    setAPIDTarget("intake_lift_PID", 0);
    runAutoPilot(100);
    setAPIDIsActivated("intake_lift_PID", false);
    setAPIDIsActivated("ramp_PID", false);
    setMotorPosition(intake_lift_mtr, 0, 255, MOVE_DEGREES);
    setMotorPosition(ramp_mtr, 0, 255, MOVE_DEGREES);

    pros::delay(10);
}

void stack(int blockNum) { //Alec's stacking method
  lTarget = left_mtr_back.get_position();
  rTarget = right_mtr_back.get_position();
  if (blockNum > 4) {
    left_intake.move(15 * blockNum-1);
    right_intake.move(15 * blockNum-1);
  }
  setMotorPosition(ramp_mtr, 80 * 84/6, 500, MOVE_DEGREES);
  left_intake.move(0);
  right_intake.move(0);
  pros::delay(1000);
  autonLockWheelsIntake = true;
  moveSquares(-0.7);
  autonLockWheelsIntake = false;
  left_intake.move(0);
  right_intake.move(0);
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
  	    case(0)://backup
  	    moveSquares(-1.2);
        moveSquares(1.2);
  	    pros::delay(700);
  	    if (mode ==  1) {
  	      extendRampAndMoveSquares(0);
  	    }

  	    break;

  	    case(1): //blue left
        extendRampAndMoveSquares(0.3);
  	  	left_intake.move(255);
  	  	right_intake.move(255);
  	  	moveSquares(1.6);
  	  	left_intake.move(0);
  	  	right_intake.move(0);
  	  	moveSquares(-0.9);
  	  	turn(-135 ,150);
  	  	moveSquares(1.1);
  	  	stack(4);


  	    break;

  	    case(2): //blue right

  	  	extendRampAndMoveSquares(0.2);
  	    turn(-90,100);
  	    moveSquares(0.33333333333);
  	    turn(90,100);
  	    left_intake.move(255);
  	    right_intake.move(255);
  	    moveSquares(1.2);
  	    pros::delay(500);
  	    moveSquares(0.6);
  	    pros::delay(5);
  	    left_intake.move(0);
  	    right_intake.move(0);
  	    moveSquares(-1.8);
  	    turn(90,100);
  	    moveSquares(1.21);
  	    stack(5);


  	    break;


  	    case(3): //red left
  	    extendRampAndMoveSquares(0.2);
  	    turn(90,100);
  	    moveSquares(0.33333333333);
  	    turn(-90,100);
  	    left_intake.move(255);
  	    right_intake.move(255);
  	    moveSquares(1.2);
  	    pros::delay(500);
  	    moveSquares(0.6);
  	    pros::delay(5);
  	    left_intake.move(0);
  	    right_intake.move(0);
  	    moveSquares(-1.8);
  	    turn(-90,100);
  	    moveSquares(1.21);
  	    stack(5);
  	    break;

  	    case(4): //red right
        extendRampAndMoveSquares(0.3);
        left_intake.move(255);
        right_intake.move(255);
        moveSquares(1.6);
        left_intake.move(0);
        right_intake.move(0);
        moveSquares(-0.9);
        turn(135 ,150);
        moveSquares(1.1);
        stack(4);
  	    break;

  	    case(5): //blue left 8 stak
  	    extendRampAndMoveSquares(0.3);
  	  	turn(-55 * SIDE_LEFT,100);

  	  	left_intake.move(255);
  	  	right_intake.move(255);
  	  	moveSquares(0.5);
  	  	pros::delay(5);
  	  	turn(55 * SIDE_LEFT,100);
  	  	moveSquares(1);
  	  	left_intake.move(0);
  	  	right_intake.move(0);
  	  	moveSquares(-1);
  	  	turn(90,100);
  	  	moveSquares(1);
  	  	turn(-90,100);
  	  	left_intake.move(255);
  	  	right_intake.move(255);
  	  	moveSquares(0.7);
  	  	pros::delay(500);
  	  	moveSquares(0.3);
  	  	turn(-130,100);
  	  	moveSquares(2.2);
  	  	stack(8);
  	    break;

  	    case(6): //red right 8 stak
  	    extendRampAndMoveSquares(0.3);
  	  	turn(55 * SIDE_LEFT,100);

  	  	left_intake.move(255);
  	  	right_intake.move(255);
  	  	moveSquares(0.5);
  	  	pros::delay(5);
  	  	turn(-55 * SIDE_LEFT,100);
  	  	moveSquares(1);
  	  	left_intake.move(0);
  	  	right_intake.move(0);
  	  	moveSquares(-1);
  	  	turn(-90,100);
  	  	moveSquares(1);
  	  	turn(90,100);
  	  	left_intake.move(255);
  	  	right_intake.move(255);
  	  	moveSquares(0.7);
  	  	pros::delay(500);
  	  	moveSquares(0.3);
  	  	turn(130,100);
  	  	moveSquares(2.2);
  	  	stack(8);
  	    break;

  	    case(7): //stack
  	      stack(10);
  	      break;



  	    case(8): //skills

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
