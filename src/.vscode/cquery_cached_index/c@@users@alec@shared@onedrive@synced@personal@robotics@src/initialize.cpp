#include "Settings.h"
#include "generalFunctions.h"

void displayTeamName() {
	  pros::lcd::set_text(2, "7701-Bread");
}

void LEDs(bool state) {
	pros::c::adi_digital_write(ports::LED_1, state);
	pros::c::adi_digital_write(ports::LED_2, not state);

}

//std::string name1, int drive_type1, int CANNON_UP1, int CANNON_DOWN1, int FIRE_CANNON1, int HIGH_FLIP_UP1, int HIGH_FLIP_DOWN1,int AUTON, int LED_CONFIG1) {

//drivers
driver Alec{"Alec \"Maverick\" Pannunzio",0,DIGITAL_L1,DIGITAL_L2,DIGITAL_R1,DIGITAL_UP,DIGITAL_DOWN,DIGITAL_X,0};
driver Gian{"Gian \"Franko\" Gonzales",2,DIGITAL_L1,DIGITAL_L2,DIGITAL_R1,DIGITAL_UP,DIGITAL_DOWN,DIGITAL_X,0};
driver Josh{"Joshua \"Kiki-Jiki\" Satterfield",1,DIGITAL_L1,DIGITAL_L2,DIGITAL_R1,DIGITAL_UP,DIGITAL_DOWN,DIGITAL_X,0};
driver Steven{"Steven",1,DIGITAL_R1,DIGITAL_R2,DIGITAL_R1,DIGITAL_R1,DIGITAL_R2,DIGITAL_LEFT,0};


driver getDriverOb(int dIndx) {
	switch(dIndx) {
		case(0):
			return Alec;
			break;
		case(1):
			return Gian;
			break;
		case(2):
			return Josh;
			break;
		case(3):
			return Steven;
			break;
		default:
			return Alec;
			break;
	}
}


int auton_select = 0;
int driverIndex = 0;
driver Driver = Alec;

int getAuton() {
	return auton_select;
}

driver getDriver() {
	return Driver;
}

bool auton_selected = false,driver_selected = false;;

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		if (not auton_selected) { //select auton
			pros::lcd::clear_line(4);
			pros::lcd::set_text(4, "Auton selected");
			auton_selected = true;
			pros::lcd::clear_line(5);
			pros::lcd::clear_line(6);
			LEDs(true);
			pros::delay(100);
			LEDs(false);
			pros::delay(200);
			LEDs(true);
			pros::delay(100);
			LEDs(false);
			std::string autonStr = "Auton: " + std::to_string(auton_select);
			pros::lcd::set_text(4,autonStr);
			std::string message = "select driver: " + getDriverOb(driverIndex).name;
			pros::lcd::set_text(5,message);
		}else if (not driver_selected){
			driver_selected = true;
			Driver = getDriverOb(driverIndex);
			pros::lcd::clear_line(5);
			pros::lcd::set_text(5,"driver selected");
			LEDs(true);
			pros::delay(100);
			LEDs(false);
			pros::delay(200);
			LEDs(true);
			pros::delay(100);
			LEDs(false);
			pros::lcd::clear_line(5);
			std::string message = "Driver: " + Driver.name;
			pros::lcd::set_text(5,message);
		}

	}
}

void on_left_button() {
	if (not auton_selected) {
		auton_select--;
		std::string a_s_str1 = std::to_string(auton_select);
		pros::lcd::set_text(6, a_s_str1);
	}else if (not driver_selected){
		driverIndex--;
		pros::lcd::clear_line(5);
		std::string message = "select driver: " + getDriverOb(driverIndex).name;
		pros::lcd::set_text(5,message);
	}
}

void on_right_button() {
	if (not auton_selected) {
		auton_select++;
		pros::lcd::clear_line(6);
		std::string a_s_str2 = std::to_string(auton_select);
		pros::lcd::set_text(6, a_s_str2);
	}else if (not driver_selected){
		driverIndex++;
		pros::lcd::clear_line(5);
		std::string message = "select driver: " + getDriverOb(driverIndex).name;
		pros::lcd::set_text(5,message);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */



void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "The JAVA MASTER is in command");
	pros::lcd::set_text(5,"select auton\\/");
	std::string a_s_str0 = std::to_string(0);
	pros::lcd::set_text(6, a_s_str0);

	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn2_cb(on_right_button);

   pros::c::adi_port_set_config(ports::BALL_FIRE_PORT,pros::E_ADI_DIGITAL_OUT);
	 pros::c::adi_port_set_config(ports::CANNON_AIM_PORT,pros::E_ADI_DIGITAL_OUT);

	 //LED ports
	 pros::c::adi_port_set_config(ports::LED_1,pros::E_ADI_DIGITAL_OUT);
	 pros::c::adi_port_set_config(ports::LED_2,pros::E_ADI_DIGITAL_OUT);
	 pros::c::adi_port_set_config(ports::LED_3,pros::E_ADI_DIGITAL_OUT);
	 pros::c::adi_port_set_config(ports::LED_4,pros::E_ADI_DIGITAL_OUT);


}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

	while(true) {
		LEDs(false);
		pros::delay(1000);
		LEDs(true);
		pros::delay(100);
	}

}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}
