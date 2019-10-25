#include "Settings.h"
#include "generalFunctions.h"
#include "api.h"


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */



 pros::ADIPotentiometer sensor (ports::DRIVE_POTENTIOMETER_PORT);

 int getDrive() {

  	int arcade;
    int PotAuton = sensor.get_value();
  	if (PotAuton <= 1500) {
  			arcade = 0;
  			pros::lcd::set_text(3, "Arcade");
  		} else if (PotAuton <= 3000) {
  			arcade = 1;
  			pros::lcd::set_text(3, "Tank");
  		}else if (PotAuton >= 3000){
  			arcade = 2;
  		  pros::lcd::set_text(3, "Drone");
  		}

  		return arcade;
  }

void opcontrol() {

  //controller
	pros::Controller master(pros::E_CONTROLLER_MASTER);

  //wheels
	pros::Motor left_mtr_back(ports::LEFT_WHEEL_BACK_PORT,pros::E_MOTOR_GEARSET_18,0,pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor right_mtr_back(ports::RIGHT_WHEEL_BACK_PORT,pros::E_MOTOR_GEARSET_18,1,pros::E_MOTOR_ENCODER_DEGREES);

	pros::Motor left_mtr_front(ports::LEFT_WHEEL_FRONT_PORT,pros::E_MOTOR_GEARSET_18,0,pros::E_MOTOR_ENCODER_DEGREES);
	pros::Motor right_mtr_front(ports::RIGHT_WHEEL_FRONT_PORT,pros::E_MOTOR_GEARSET_18,1,pros::E_MOTOR_ENCODER_DEGREES);


  //conveyor
  pros::Motor aim_mtr(ports::CANNON_AIM_PORT);

	//high flipper
	pros::Motor high_flipper_mtr(ports::HIGH_FLIPPER_PORT,pros::E_MOTOR_GEARSET_18,1,pros::E_MOTOR_ENCODER_ROTATIONS);

	//ultrasonic sensor
//	pros::ADIUltrasonic ultrasonic(ports::ULTRASONIC_ECHO_PORT,ports::ULTRASONIC_PING_PORT);


bool potDrivetrainSelect = false;

driver driver = getDriver();

int drive;

unsigned long loops = 0;

while (true) {
    driver = getDriver();

    if (potDrivetrainSelect) {
      drive = getDrive();
    }else{
      drive = driver.drive_type;
    }

		LEDs(false);

		pros::lcd::set_text(2, "7701-Bread");

    //display what loop we are on
    // std::string loops_str = std::to_string(loops);
    // pros::lcd::set_text(7, loops_str);

		checkForStop();


		int left, right, straif, s_correction;
		if(drive == 0) { //arcade drive
			int power = master.get_analog(ANALOG_LEFT_Y);
			int turn = master.get_analog(ANALOG_LEFT_X);
			left = power + turn;
			right = power - turn;
			s_correction = (master.get_analog(ANALOG_RIGHT_Y));
      straif = (master.get_analog(ANALOG_RIGHT_X));
		}else if (drive == 1){ //tank drive
			left = master.get_analog(ANALOG_LEFT_Y);
			right = master.get_analog(ANALOG_RIGHT_Y);
      straif = (master.get_analog(ANALOG_RIGHT_X));
			s_correction = 0;
		}else if (drive == 2) { //drone drive
      int power = master.get_analog(ANALOG_RIGHT_Y);
      int turn = master.get_analog(ANALOG_LEFT_X);
      left = power + turn;
      right = power - turn;
      s_correction = 0;
      straif = (master.get_analog(ANALOG_RIGHT_X));
    }

		left_mtr_back.move(straif + left);
		right_mtr_back.move(-straif + right);
		left_mtr_front.move(-straif*.1*(0.02*s_correction + 1) + left);
		right_mtr_front.move(straif*.1*(0.02*s_correction + 1) + right);

		if( ( abs(left)+abs(right)+abs(2*straif) ) > 100) {
		 LEDs(true);
		}

    //high flipper
    if (master.get_digital(driver.HIGH_FLIP_UP)) {
        high_flipper_mtr = 200;
    }else if (master.get_digital(driver.HIGH_FLIP_DOWN)) {
			  high_flipper_mtr =0;
    }else{

		}

    //aim motor
    if (master.get_digital(driver.CANNON_UP)) {
        aim_mtr = 50;
    }else if (master.get_digital(driver.CANNON_DOWN)) {
       aim_mtr = -50;
    }else {
      aim_mtr = 0;
    }

    //launcher
    if (master.get_digital(driver.FIRE_CANNON)) {
			 LEDs(true);
       pros::c::adi_digital_write(ports::BALL_FIRE_PORT, true);
    }else {
       pros::c::adi_digital_write(ports::BALL_FIRE_PORT, false);
    }


		//auton button
		if (master.get_digital(driver.LAUNCH_AUTON)) {
			LEDs(true);
			pros::delay(100);
			LEDs(false);
			autonomous();
		}

    //blink lights
    /*
		if (loops % 150 == 0) {
			LEDs(true);
		}else if (loops % 150 == 90) {
			LEDs(false);
		}
    */


		pros::delay(10);


		loops++;
	}
}
