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

#include <algorithm>
#include "vex.h"

#include "../../../Utils/nutils.hpp"
#include "../../../Utils/DriveBase.hpp"

using namespace vex;
competition Competition;

// use heading(), data range [0, 360]. so calibrate to 0 means the initial reading can be 365.5 !
auto ss0 = ScrollingScreen<int>(Brain, 1, 2);
auto ss = ScrollingScreen<int>(Brain, 3, 10);

auto db = DriveBase(backleftdrive, backrightdrive, frontleftdrive, frontrightdrive, ss, inertialSensor);

double computeDistanceForOneRotation() {
  // return the wheel travel distance (in inch) when motor spins one rotation
  double wheelDiameter = 4.0; // omni "green" wheel
  double wheelToDriveGearRatio = 1.0;
  
  return 3.14159265 * wheelDiameter / wheelToDriveGearRatio;
}


double computeRotationsFromDistance(double x /*inch*/) {
  return x / computeDistanceForOneRotation();
}








void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  ss.print("Hello, 47114A! Program to start.\n");

}


void autonomous( void ) {
  double kp = 0.0;
  double dist = 4 * 24;
  double backoutDist = 1.45 * 24;
  double dist2 = 0.5 * 24;
  ss.print("Go straight ahead for %4.1f inches, PID kp=%4.1f\n", dist, kp);

  double intakeSpeed = 60;

  leftintake.setVelocity(intakeSpeed, percentUnits::pct);
  rightintake.setVelocity(intakeSpeed, percentUnits::pct);

  leftintake.spin(directionType::rev);
  rightintake.spin(directionType::rev);
  db.goStraight(computeRotationsFromDistance(dist), 40, 10, 10, kp, false, true);
  
  vex::task::sleep(500);

  leftintake.stop(brakeType::hold);
  rightintake.stop(brakeType::hold);

  ss.print("Go reverse");
  db.goStraight(computeRotationsFromDistance(backoutDist), 40, 10, 10, kp, true, true);
  //vex::task::sleep(500);

  ss0.print("Make left turn");
  db.makeTurn(20, 5, 0, true);
  //vex::task::sleep(500);

  // ss0.print("Make right turn");
  // makeTurn(15, 5, 0, false);
  // vex::task::sleep(1000);

  // ss0.print("Make 2nd left turn");
  //   makeTurn(15, 5, 0, true);
  // vex::task::sleep(500);

  // ss0.print("Make 2nd right turn");
  // makeTurn(15, 5, 0, false);
  // vex::task::sleep(1000);

  // 2nd stage go straight (to left)
  ss.print("Go straight (to left)");
  db.goStraight(computeRotationsFromDistance(dist2), 40, 10, 10, 0.0, false, true);

  leftintake.spin(directionType::rev);
  rightintake.spin(directionType::rev);
  
  vex::task::sleep(300);

  leftintake.setBrake(brakeType::hold);
  rightintake.setBrake(brakeType::hold);
  ss.print("START: lift up");
  lift.spinFor(vex::directionType::fwd, 0.75, vex::rotationUnits::rev);
  vex::task::sleep(1000);

  // back off
  ss.print("START: go back again");
  db.goStraight(computeRotationsFromDistance(dist2), 40, 10, 10, kp, true /*reverse*/, true);
  ss.print("END: go back again");

  // 180 degree turn as two 90 left turns
  db.makeTurn(20, 5, 0, true); 
  db.makeTurn(20, 5, 0, true);


  // push balls out
  leftintake.spin(directionType::fwd);
  rightintake.spin(directionType::fwd);

  leftintake.setBrake(brakeType::hold);
  rightintake.setBrake(brakeType::hold);
  


  ss.print("DONE");
  ss.print("dist for one rot: %.2f", computeDistanceForOneRotation());
}


void usercontrol( void ) {
  ss.print("User control not impl yet");
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}
