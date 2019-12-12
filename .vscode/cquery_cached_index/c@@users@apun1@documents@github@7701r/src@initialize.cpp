#include "generalFunctions.h"
#include "api.h"
#include "Settings.h"

//controller
pros::Controller master(pros::E_CONTROLLER_MASTER), partner(pros::E_CONTROLLER_PARTNER);

//motor creation
  //wheels
 GEAH::Motor left_mtr_back("left_mtr_back",ports::LEFT_WHEEL_BACK_PORT,pros::E_MOTOR_GEARSET_18,0,pros::E_MOTOR_ENCODER_DEGREES);
 GEAH::Motor right_mtr_back("right_mtr_back",ports::RIGHT_WHEEL_BACK_PORT,pros::E_MOTOR_GEARSET_18,1,pros::E_MOTOR_ENCODER_DEGREES);

 GEAH::Motor left_mtr_front("left_mtr_front",ports::LEFT_WHEEL_FRONT_PORT,pros::E_MOTOR_GEARSET_18,0,pros::E_MOTOR_ENCODER_DEGREES);
 GEAH::Motor right_mtr_front("right_mtr_front",ports::RIGHT_WHEEL_FRONT_PORT,pros::E_MOTOR_GEARSET_18,1,pros::E_MOTOR_ENCODER_DEGREES);

 GEAH::Motor ramp_mtr("ramp_mtr",ports::RAMP_MTR_PORT,pros::E_MOTOR_GEARSET_18,0,pros::E_MOTOR_ENCODER_DEGREES);
 GEAH::Motor intake_lift_mtr("intake_lift_mtr",ports::INTAKE_LIFT_MTR_PORT,pros::E_MOTOR_GEARSET_36,0,pros::E_MOTOR_ENCODER_DEGREES);
 GEAH::Motor left_intake("left_intake",ports::INTAKE_LEFT_MTR_PORT,pros::E_MOTOR_GEARSET_18,0,pros::E_MOTOR_ENCODER_DEGREES);
 GEAH::Motor right_intake("right_intake",ports::INTAKE_RIGHT_MTR_PORT,pros::E_MOTOR_GEARSET_18,1,pros::E_MOTOR_ENCODER_DEGREES);
/*
 //angler
 GEAH::Motor aim_mtr("aim_mtr",ports::CANNON_AIM_PORT,pros::E_MOTOR_GEARSET_18,0,pros::E_MOTOR_ENCODER_DEGREES);

 //intake
 GEAH::Motor intake_mtr("intake_mtr",ports::INTAKE_PORT,pros::E_MOTOR_GEARSET_06,1,pros::E_MOTOR_ENCODER_DEGREES);

//fire motor
 GEAH::Motor cannonFireMotor("cannonFireMotor",ports::BALL_FIRE_PORT,pros::E_MOTOR_GEARSET_18,1,pros::E_MOTOR_ENCODER_DEGREES);
*/
std::vector<GEAH::Motor> Motors = {left_mtr_back,right_mtr_back,left_mtr_front,right_mtr_front,ramp_mtr,intake_lift_mtr,left_intake,right_intake};

void initializeAutoPilot();

void displayTeamName() {
	  pros::lcd::set_text(2, "7701-Bread");
}

void LEDs(bool state) {
	pros::c::adi_digital_write(ports::LED_1, state);
	pros::c::adi_digital_write(ports::LED_2, not state);

}


namespace GEAH {

  std::vector<GEAH::controllerButton> controllerButtons = {{DIGITAL_A,0,false,"notAssigned"},{DIGITAL_A,0,false,"c0A"},{DIGITAL_A,0,true,"sc0A"},{DIGITAL_B,0,false,"c0B"},{DIGITAL_B,0,true,"sc0B"},{DIGITAL_X,0,false,"c0X"},{DIGITAL_X,0,true,"sc0X"},{DIGITAL_Y,0,false,"c0Y"},{DIGITAL_Y,0,true,"sc0Y"},{DIGITAL_L1,0,false,"c0L1"},{DIGITAL_L1,0,true,"sc0L1"},{DIGITAL_L2,0,false,"c0L2"},{DIGITAL_L2,0,true,"sc0L2"},{DIGITAL_R1,0,false,"c0R1"},{DIGITAL_R1,0,true,"sc0R1"},{DIGITAL_R2,0,false,"c0R2"},{DIGITAL_R2,0,true,"sc0R2"},{DIGITAL_UP,0,false,"c0UP"},{DIGITAL_UP,0,true,"sc0UP"},{DIGITAL_DOWN,0,false,"c0DOWN"},{DIGITAL_DOWN,0,true,"sc0DOWN"},{DIGITAL_LEFT,0,false,"c0LEFT"},{DIGITAL_LEFT,0,true,"sc0LEFT"},{DIGITAL_RIGHT,0,false,"c0RIGHT"},{DIGITAL_RIGHT,0,true,"sc0RIGHT"},{DIGITAL_A,1,false,"c1A"},{DIGITAL_A,1,true,"sc1A"},{DIGITAL_B,1,false,"c1B"},{DIGITAL_B,1,true,"sc1B"},{DIGITAL_X,1,false,"c1X"},{DIGITAL_X,1,true,"sc1X"},{DIGITAL_Y,1,false,"c1Y"},{DIGITAL_Y,1,true,"sc1Y"},{DIGITAL_L1,1,false,"c1L1"},{DIGITAL_L1,1,true,"sc1L1"},{DIGITAL_L2,1,false,"c1L2"},{DIGITAL_L2,1,true,"sc1L2"},{DIGITAL_R1,1,false,"c1R1"},{DIGITAL_R1,1,true,"sc1R1"},{DIGITAL_R2,1,false,"c1R2"},{DIGITAL_R2,1,true,"sc1R2"},{DIGITAL_UP,1,false,"c1UP"},{DIGITAL_UP,1,true,"sc1UP"},{DIGITAL_DOWN,1,false,"c1DOWN"},{DIGITAL_DOWN,1,true,"sc1DOWN"},{DIGITAL_LEFT,1,false,"c1LEFT"},{DIGITAL_LEFT,1,true,"sc1LEFT"},{DIGITAL_RIGHT,1,false,"c1RIGHT"},{DIGITAL_RIGHT,1,true,"sc1RIGHT"}};

  bool buttonIsPressed(GEAH::controllerButton button); //(in opcontroll.cpp)
}



GEAH::controllerButton getButtonObject(std::string name) {

  for (GEAH::controllerButton but : GEAH::controllerButtons) {
    if (but.name == name) {
      return but;
    }
  }
  return GEAH::controllerButtons[0];
}

//  (name, drive_type,n shiftButton, ramp_up, ramp_down,intake_up, intake_down, intake_in, intake_out,lockWheelsIntake, LAUNCH_AUTON, BREAKS_ON, BREAKS_OFF)

//drivers
namespace GEAH {
driver Alec{"Alec \"Maverick\" Pannunzio",0,getButtonObject("c0X"),getButtonObject("c0R1"),getButtonObject("c0R2"),getButtonObject("c0UP"),getButtonObject("c0DOWN"),getButtonObject("c0L1"),getButtonObject("c0L2"),getButtonObject("sc0L2"),getButtonObject("sc0UP"),getButtonObject("notAssigned"),getButtonObject("notAssigned")};
driver Hayden{"Hayden \"Xerminator13\" Corbin",1,getButtonObject("c0X"),getButtonObject("c0L1"),getButtonObject("c0L2"),getButtonObject("c0UP"),getButtonObject("c0DOWN"),getButtonObject("c0R1"),getButtonObject("c0R2"),getButtonObject("sc0L1"),getButtonObject("sc0UP"),getButtonObject("c0B"),getButtonObject("sc0B")};
driver Jay{"Jay \"TheCoolest\" ",0,getButtonObject("c0X"),getButtonObject("c0R1"),getButtonObject("c0R2"),getButtonObject("c0UP"),getButtonObject("c0DOWN"),getButtonObject("c0L1"),getButtonObject("c0L2"),getButtonObject("sc0L2"),getButtonObject("sc0UP"),getButtonObject("notAssigned"),getButtonObject("notAssigned")};

}

GEAH::driver getDriverOb(int dIndx) {
	switch(dIndx) {
		case(0):
			return GEAH::Alec;
			break;
    case(1):
      return GEAH::Hayden;
      break;
    case(2):
      return GEAH::Jay;
      break;

		default:
			return GEAH::Alec;
			break;
	}
}


int auton_select = 0;
int driverIndex = 0;
GEAH::driver Driver = GEAH::Alec;



GEAH::driver getDriver() {
	return Driver;
}

bool auton_selected = false,driver_selected = false;
std::vector<std::string> autonNames = {"backup","left-near","right-near","left-far-park","right-far-park","far-noPark","exFarLeft","exFarRight","Skills","callibration","none"};

int getAuton() {
	return auton_select;
}

int getAuton(std::string autonName) {
  for (int i = 0; i < autonNames.size(); i++) {
    if (autonNames.at(i) == autonName) {
      return i;
    }
  }
  if (autonName != "none") {
    return getAuton("none");
  }else{
    return -1;
  }
}

std::string getAutonName(int autSel) {
  if ((autSel >= 0) && (autSel < autonNames.size())) {
    return autonNames.at(autSel);
  }else{
    return std::to_string(auton_select);
  }
}

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
			std::string autonStr = "Auton: " + getAutonName(auton_select);
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
		std::string a_s_str1 = getAutonName(auton_select);
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
		std::string a_s_str2 = getAutonName(auton_select);
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


void remoteSettingSelect() {

		while(not (auton_selected && driver_selected)) {
			if (master.get_digital(DIGITAL_LEFT)) {
				on_left_button();
				vibrateController(".");
			}else if (master.get_digital(DIGITAL_RIGHT)) {
				on_right_button();
				vibrateController(".");
			}else if (master.get_digital(DIGITAL_A)) {
				on_center_button();
				vibrateController("-");
			}

			if (not auton_selected) {
				std::string aut_char = std::to_string(auton_select);
				master.set_text(0, 0, "A");
				master.set_text(0, 1, "t");
				master.set_text(0, 2, "n");
				master.set_text(0, 3, ":");
				switch(auton_select) {
					case (0):
						master.set_text(1, 0, "0");
						break;
				}
			}else if (not driver_selected) {


				master.set_text(0, 0, "D");
				master.set_text(0, 1, "v");
				master.set_text(0, 2, "r");
				master.set_text(0, 3, ":");

			}
			pros::delay(200); //the time this is may cause repeated button_pressed calls on one press or presses not being recorded
		}


}

bool runCallibration();
void runAutoPilot(int times);

void initialize() {


	pros::lcd::initialize();
	pros::lcd::set_text(1, "The JAVA MASTER is in command");
	pros::lcd::set_text(5,"select auton\\/");
	std::string a_s_str0 = getAutonName(0);
	pros::lcd::set_text(6, a_s_str0);

	pros::lcd::register_btn1_cb(on_center_button);
	pros::lcd::register_btn0_cb(on_left_button);
	pros::lcd::register_btn2_cb(on_right_button);





	 //LED ports
	 pros::c::adi_port_set_config(ports::LED_1,pros::E_ADI_DIGITAL_OUT);
	 pros::c::adi_port_set_config(ports::LED_2,pros::E_ADI_DIGITAL_OUT);
	 pros::c::adi_port_set_config(ports::LED_3,pros::E_ADI_DIGITAL_OUT);
	 pros::c::adi_port_set_config(ports::LED_4,pros::E_ADI_DIGITAL_OUT);

	 remoteSettingSelect();


   //initializing autoPilot
   initializeAutoPilot();


   pros::delay(10);
	 //zeroing motors


   //tareing driveTrain
   left_mtr_back.tare_position();
   right_mtr_back.tare_position();
   left_mtr_front.tare_position();
   right_mtr_front.tare_position();


   left_mtr_back.set_brake_mode(MOTOR_BRAKE_BRAKE);
   right_mtr_back.set_brake_mode(MOTOR_BRAKE_BRAKE);
   left_mtr_front.set_brake_mode(MOTOR_BRAKE_BRAKE);
   right_mtr_front.set_brake_mode(MOTOR_BRAKE_BRAKE);


  setDriveTrainPIDIsActivated(false);
  setAPIDIsActivated("intake_lift_PID", false);
  pros::delay(10);

  std::cout << "Successfully initialized.\n";

  if (autonNames.at(auton_select) == "callibration") {
    std::cout << "running callbration\n";
    runCallibration();
  }
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {

	remoteSettingSelect();



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
void competition_initialize() {

  remoteSettingSelect();

  while(true) {
		LEDs(false);
		pros::delay(1000);
		LEDs(true);
		pros::delay(100);
	}
}
