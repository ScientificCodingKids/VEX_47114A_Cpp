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

#include "vex.h"
#include "../../../Utils/nutils.hpp"

using namespace vex;
competition Competition;

// use heading(), data range [0, 360]. so calibrate to 0 means the initial reading can be 365.5 !

double x = 0.5;

auto ss = ScrollingScreen<int>();


void turnleft(void) {
  // auto rotation using PID and inertial sensor
  inertialSensor.calibrate();

  while (inertialSensor.isCalibrating()) {
    vex::wait(50, timeUnits::msec);
  }


  inertialSensor.resetRotation();
  leftDriveMotor.resetPosition();
  rightDriveMotor.resetPosition();

  double origSpeed = 10;
  double speed = 0.0;
  Brain.Screen.clearScreen();
  Brain.Screen.print("Start inertia sensor run \n");
  Brain.Screen.print("Initial: %4.1f \n", inertialSensor.rotation());
  Brain.Screen.newLine();

  while ((inertialSensor.heading() < 90.0) || (inertialSensor.heading() > 180)) {
    speed = origSpeed;
    // slow start

    double r = 360 - inertialSensor.heading(); // this is to make sure we get a small, negative number.

    if (r > 180) {
      r = r - 360;
    }

    if (r < 10.0) {
      speed = origSpeed * std::max(0.3, r / 10.0);

    }

    if (r > 80.0) {
      speed = origSpeed * (1.0 - (r - 80.0)/ 10.0) * 1.3;
    }

    leftDriveMotor.setVelocity(speed, vex::percentUnits::pct);

    rightDriveMotor.setVelocity(speed, vex::percentUnits::pct);  // not moving !!!

    leftDriveMotor.spin(directionType::rev);
    rightDriveMotor.spin(directionType::fwd);

    Brain.Screen.print("Rotate %4.1f , %4.1f\n", r, speed);
    Brain.Screen.newLine();
    vex::task::sleep(1);
  } // while

  leftDriveMotor.stop();
  rightDriveMotor.stop();
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}
