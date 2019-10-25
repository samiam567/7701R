#include "generalFunctions.h"
#include "graphics.h"
#include "Settings.h"

extern pros::Controller master;
std::string remoteSettingSelect(std::string title, std::vector<std::string> choices, std::string outputType) {
  int currentSelection = 0;
  bool selected = false;
  bool waitingForButtonPress = false;

  while (! selected) {

    //outputing current selection and prompting the user
    if (outputType == "console") {
      consoleClear();
      consoleLogN("Choose " + title + ":");
      consoleLogN(choices.at(currentSelection));
      consoleLogN("Use arrows and \"A\" to select");
    }else if (outputType == "cout"){
      std::cout << ("Choose " + title + ":") << "\n";
      std::cout << (choices.at(currentSelection)) << "\n";
      std::cout << ("Use arrows and \"A\" to select") << "\n";
    }else{
      std::cout << "invalid outputType in remoteSettingSelect" << "\n";
    }

    waitingForButtonPress = true;

    while (waitingForButtonPress) {
      if (master.get_digital(DIGITAL_A)) {
        waitingForButtonPress = false;
        selected = true;
        return choices.at(currentSelection);
      }else if (master.get_digital(DIGITAL_LEFT)) {
        waitingForButtonPress = false;
        currentSelection--;
      }else if (master.get_digital(DIGITAL_RIGHT)) {
        waitingForButtonPress = false;
        currentSelection++;
      }
      pros::delay(10);
    }
    vibrateController(".");
    pros::delay(100);

    //making sure the currentSelection is within the bounds of the choices
    if (currentSelection < 0) {
      currentSelection = 0;
    }else if (currentSelection >= choices.size()) {
      currentSelection = choices.size()-1;
    }


  }
  std::cout << "Error in  remoteSettingSelect: reached end of function without a selection being made";
  return " ";
}
