#include <cmath>
#include "main.h"


std::vector<double> rectangularToPolar(double xComponent, double yComponent) {
  std::vector<double> rot1List = {sqrt(xComponent*xComponent + yComponent*yComponent),atan2(yComponent,xComponent)};
  return rot1List;
}

std::vector<double> polarToRectangular(double r, double theta) {
  std::vector<double> rot2List = {r * cos(theta),r * sin(theta)};
  return rot2List;
}

std::vector<double> calculateRotation(double xComponent, double yComponent, double angle) {
  std::vector<double> polar = rectangularToPolar(xComponent, yComponent);
	return polarToRectangular(polar[0], polar[1] + angle);
}
