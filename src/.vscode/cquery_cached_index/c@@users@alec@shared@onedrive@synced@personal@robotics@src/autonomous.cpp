#include "Settings.h"
#include "generalFunctions.h"

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

//motor creation

  //wheels
      //note: the encoder value is different here than opControl
 pros::Motor left_mtr_back(ports::LEFT_WHEEL_BACK_PORT,pros::E_MOTOR_GEARSET_18,0,pros::E_MOTOR_ENCODER_DEGREES);
 pros::Motor right_mtr_back(ports::RIGHT_WHEEL_BACK_PORT,pros::E_MOTOR_GEARSET_18,1,pros::E_MOTOR_ENCODER_DEGREES);

 pros::Motor left_mtr_front(ports::LEFT_WHEEL_FRONT_PORT,pros::E_MOTOR_GEARSET_18,0,pros::E_MOTOR_ENCODER_DEGREES);
 pros::Motor right_mtr_front(ports::RIGHT_WHEEL_FRONT_PORT,pros::E_MOTOR_GEARSET_18,1,pros::E_MOTOR_ENCODER_DEGREES);


 //conveyor
 pros::Motor aim_mtr(ports::CANNON_AIM_PORT,pros::E_MOTOR_GEARSET_18,1,pros::E_MOTOR_ENCODER_DEGREES);

 //high flipper
 pros::Motor high_flipper_mtr(ports::HIGH_FLIPPER_PORT,pros::E_MOTOR_GEARSET_18,1,pros::E_MOTOR_ENCODER_DEGREES);


void checkForStop() {
  //check if moving parts are going to hit something
}


void driveStraight(float magnatude, int speed, int type) {
		//for type:
		//0 = rotations
		//1 = meters

    float give = 0.5f;

		float wheelRadius = 0.05305; //radius of the wheels in meters

		float wheelRotations;

		if (type == 0) {
			wheelRotations = magnatude;
		}else if (type == 1) {
			wheelRotations = magnatude / ( 2 * wheelRadius * M_PI);
		}else{
			//default to distance
			wheelRotations = magnatude / ( 2 * wheelRadius * M_PI);
		}

		magnatude = fabs(magnatude);


		double lValue,lStart;
		double rValue,rStart;
		lValue = 0.000001;
		rValue = 0.000001;

    lStart = left_mtr_back.get_position();
    rStart = right_mtr_back.get_position();

		float correctionValue;



		float degs = fabs(360*wheelRotations);

		while (lValue < degs ) {


			checkForStop(); //this method should be continuously called the entire duration of the program

			lValue = fabs(left_mtr_back.get_position()-lStart);
			rValue = fabs(right_mtr_back.get_position()-rStart);


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

}

void turn(float theta, int speed) { //theta is in degrees
		float wheelRadius = 0.05305;
  	float radiusOfRotation = 0.1825;
    theta = 2 * M_PI * theta/360;
  	float wheelDistance = radiusOfRotation * theta;

  	float wheelRotations = fabs(wheelDistance / ( 2 * wheelRadius * M_PI));

  	int lMulti;
  	int rMulti;

  	if (theta >= 0) {
  		lMulti = -1;
  		rMulti = 1;
  	}else{
  		lMulti = 1;
  		rMulti = -1;
  	}


		double lValue,lStart;
		double rValue,rStart;
		lValue = 0.000001;
		rValue = 0.000001;

		float correctionValue;

    lStart = left_mtr_back.get_position();
		rStart = right_mtr_back.get_position();

		while (rValue < 360 * wheelRotations ) {

			checkForStop(); //this method should be continuously called the entire duration of the program
			lValue = fabs(left_mtr_back.get_position() - lStart);
			rValue = fabs(right_mtr_back.get_position() - rStart);


			if (lValue == rValue) { //the sides are equal and no correction needs to be made
        left_mtr_back = lMulti * speed;
        left_mtr_front = lMulti * speed;

			  right_mtr_back = rMulti * speed;
        right_mtr_front = rMulti * speed;


			}else if (lValue > rValue) { // the left side is going faster than the right and needs to be slowed down
				correctionValue = speed * callibrationSettings::TURN_CORRECTION;

        left_mtr_back = lMulti * correctionValue;
        left_mtr_front = lMulti * correctionValue;

			  right_mtr_back = rMulti * speed;
        right_mtr_front = rMulti * speed;


			}else if (lValue < rValue) { // the left side is going faster than the right and needs to be slowed down
				correctionValue = callibrationSettings::TURN_CORRECTION * speed;

        left_mtr_back = lMulti * speed;
        left_mtr_front = lMulti * speed;

        right_mtr_back = rMulti * correctionValue;
        right_mtr_front = rMulti * correctionValue;

			}

		}

    //reversing the motors for a bit to counteract the rotational inertia of the bot
    left_mtr_back = - lMulti * speed;
    left_mtr_front = - lMulti * speed;

    right_mtr_back = - rMulti * speed;
    right_mtr_front = - rMulti * speed;

		pros::delay(100);

    left_mtr_back = 0;
    left_mtr_front = 0;

    right_mtr_back = 0;
    right_mtr_front = 0;
}

void moveMotor(pros::Motor motor, float magnatude, int speed, int type) {
    //for type:
    //0 = rotations
    //1 = meters

    float give = 0.5f;

    float wheelRadius = 0.05305; //radius of the wheels in meters

    float wheelRotations;

    if (type == 0) {
      wheelRotations = magnatude;
    }else if (type == 1) {
      wheelRotations = magnatude / ( 2 * wheelRadius * M_PI);
    }else{
      //default to distance
      wheelRotations = magnatude / ( 2 * wheelRadius * M_PI);
    }

    magnatude = fabs(magnatude);


    double value,start;

    value = 0.000001;

    start = motor.get_position();


    float degs = fabs(360*wheelRotations);

    while (value < degs ) {
        checkForStop(); //this method should be continuously called the entire duration of the program

        value = fabs(motor.get_position()-start);

        motor = speed;
    }

    //set all motors to zero after the bot has gone the specified amount
      motor = 0;
}

void setMotorPosition(pros::Motor motor, float position, int speed, int type) {
  //for type:
  //0 = rotations
  //1 = degrees

  if (type == 0) { //convert rotations to degrees
    position *= 360;
  }

  if (abs(motor.get_position() - position) < callibrationSettings::MOTOR_POSITION_ERROR) {
    motor = 0;
  }else if (motor.get_position() < position) {
    motor = speed;
  }else{
    motor = -speed;
  }

}


void autonomous() {

  LEDs(true);
  pros::delay(100);
  LEDs(false);

  switch(getAuton()) {

    case(0):
      LEDs(true);
      moveMotor(aim_mtr,0.8,50,0);
      LEDs(false);
      pros::delay(600);
      LEDs(true);
      pros::c::adi_digital_write(ports::BALL_FIRE_PORT, true);
      pros::delay(50);
      pros::c::adi_digital_write(ports::BALL_FIRE_PORT, false);
      LEDs(false);
      pros::delay(800);
      LEDs(true);
      driveStraight(1,50,1);
    break;

    case(1):
      driveStraight(0.6,50,1);
      turn(-90,50);
      driveStraight(-0.2,-100,1);
      driveStraight(1.5,300,1);
    break;

    case(2):
      driveStraight(0.6,50,1);
      turn(90,50);
      driveStraight(-0.2,-100,1);
      driveStraight(1.5,300,1);
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

}
