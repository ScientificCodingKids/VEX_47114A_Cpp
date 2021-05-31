/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\King                                             */
/*    Created:      Sat Jan 23 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

// test if negative speed will result in opposite direction movement

#include "vex.h"
#include <algorithm>


#include "../../../Utils/DriveBase.hpp"
#include "../../../Utils/nutils.hpp"


using namespace vex;
competition Competition;

// use heading(), data range [0, 360]. so calibrate to 0 means the initial
// reading can be 365.5 !
auto ss0 = ScrollingScreen<int>(Brain, 1, 2);
auto ss = ScrollingScreen<int>(Brain, 3, 10);

auto db = DriveBase(backleftdrive, backrightdrive, frontleftdrive,
                    frontrightdrive, ss, inertialSensor);

double computeDistanceForOneRotation() {
  // return the wheel travel distance (in inch) when motor spins one rotation
  double wheelDiameter = 4.0; // omni "green" wheel
  double wheelToDriveGearRatio = 1.0;

  return 3.14159265 * wheelDiameter / wheelToDriveGearRatio;
}

double computeRotationsFromDistance(double x /*inch*/) {
  return x / computeDistanceForOneRotation();
}

void pre_auton(void) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  ss.print("Hello, 47114A! Program to start.\n");
}

void autonomous(void) {
  inertialSensor.calibrate();
  ss.print("%f", inertialSensor.roll());
  auto d = 10; // distance to approx. center of board

// void goStraight(double rotationsToGo, double baseSpeed, double minStartSpeed, double minEndSpeed, double kp, bool goReverse=false, bool useGyro=false);

  db.goStraight(computeRotationsFromDistance(d), 40, 20, 20, 0);

  while (std::abs(inertialSensor.roll()) > 1.5) {
    auto dir = true;
    if (inertialSensor.roll() > 0) {
      dir = true;
    }
    else {
      dir = false;
    }
    db.goStraight(computeRotationsFromDistance(0.2 * std::abs(inertialSensor.roll())), 10, 5, 5, 0, dir);
  }
  
  task::sleep(50);

}

void usercontrol(void) { ss.print("User control not impl yet"); }

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
}
