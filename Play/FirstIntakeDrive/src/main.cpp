/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Chloe Ning                                       */
/*    Created:      Wed Jan 22 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------xsssssssssss------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "../../../Utils/nutils.hpp"
#include <cmath>

using namespace vex;

competition Competition;

// auto ss = ScrollingScreen<int>();

void pre_auton( void ) {

}

void goStraight(double dist, vex::directionType dt, double tgtHeading, double speed, double kp = 0.01) {
  leftdrive.resetRotation();
  rightdrive.resetRotation();

  double distToGo = dist;

  while (distToGo > 0) {
    double headingError = inertial_sensor.heading() - tgtHeading;
    if (headingError < -270) headingError = headingError + 360;
    if (headingError > 270) headingError = headingError - 360;

    if (headingError < -15) headingError = -15;
    if (headingError > 15) headingError = 15;

    if (dt == vex::directionType::fwd) {
      leftdrive.setVelocity(speed * (1 - kp * headingError), vex::percentUnits::pct);
      rightdrive.setVelocity(speed * (1 + kp * headingError), vex::percentUnits::pct);
    } else {
      leftdrive.setVelocity(speed * (1 + kp * headingError), vex::percentUnits::pct);
      rightdrive.setVelocity(speed * (1 - kp * headingError), vex::percentUnits::pct);
    }
    leftdrive.spin(dt);
    rightdrive.spin(dt);

    vex::task::sleep(10);

    distToGo = dist - fabs(leftdrive.rotation(vex::rotationUnits::deg) / 360 * (4.0 * 3.1415269265));
  }
  leftdrive.stop();
  rightdrive.stop();
}

void autonomous( void ) {
  double tileSize = 23.5;
  double rotationSpeed = 80;

  leftintake.spinFor(vex::directionType::rev, 0.2, vex::rotationUnits::rev);
  lift.spinFor(vex::directionType::rev, 40, vex::rotationUnits::deg);

  dt.driveFor(2.0*tileSize, vex::distanceUnits::in, 90, vex::velocityUnits::pct);
  dt.driveFor(3, vex::distanceUnits::in, 40, vex::velocityUnits::pct, false);
  // sdrive.driveFor(2.0 * tileSize, vex::distanceUnits::in, 90, vex::velocityUnits::pct);
  // sdrive.driveFor(3, vex::distanceUnits::in, 40, vex::velocityUnits::pct, false);
  vex::task::sleep(200); //wait for the slow walk to complete, but don't wait for too long!!!


  // grab the 1st neutral mogo
  leftintake.setVelocity(60, vex::percentUnits::pct);
  leftintake.spinFor(vex::directionType::rev, 0.3, vex::rotationUnits::rev, false);
  vex::task::sleep(500);

  // back off with 1st neutral mogo
  lift.spinFor(vex::directionType::fwd, 200, vex::rotationUnits::deg, 70, vex::velocityUnits::pct);
  dt.driveFor(vex::directionType::rev, 40, vex::distanceUnits::in, 55, vex::velocityUnits::pct);
  // sdrive.driveFor(vex::directionType::rev, 40, vex::distanceUnits::in, 85, vex::velocityUnits::pct);

  
  // drop off the 1st mogo
  //inertial_sensor.calibrate();

  dt.turnFor(vex::turnType::right, 60, vex::rotationUnits::deg);
  lift.spinFor(vex::directionType::rev, 90, vex::rotationUnits::deg);
  leftintake.spinFor(vex::directionType::fwd, 0.3, vex::rotationUnits::rev);
}

void autonomous_old( void ) {
  double tileSize = 23.5;
  double rotationSpeed = 80;

  inertial_sensor.calibrate();
  vex::task::sleep(500);

  leftintake.spinFor(vex::directionType::rev, 0.2, vex::rotationUnits::rev);
  lift.spinFor(vex::directionType::rev, 40, vex::rotationUnits::deg);

  // aim at the 1st neutral mogo
  sdrive.turnToHeading(20, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);

  // go to the 1st neutral mogo
  goStraight(40, vex::directionType::fwd, 20, 90);
  sdrive.driveFor(3, vex::distanceUnits::in, 15, vex::velocityUnits::pct, false);
  vex::task::sleep(500); //wait for the slow walk to complete, but don't wait for too long!!!

  // grab the 1st neutral mogo
  leftintake.setVelocity(60, vex::percentUnits::pct);
  leftintake.spinFor(vex::directionType::rev, 0.3, vex::rotationUnits::rev, false);
  vex::task::sleep(500);

  // back off with 1st neutral mogo
  lift.spinFor(vex::directionType::fwd, 150, vex::rotationUnits::deg);
  sdrive.turnToHeading(0, vex::rotationUnits::deg, 50, vex::velocityUnits::pct);
  sdrive.driveFor(vex::directionType::rev, 23.5, vex::distanceUnits::in, 85, vex::velocityUnits::pct);

  // drop off the 1st mogo
  sdrive.turnToHeading(270, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);
  lift.spinFor(vex::directionType::rev, 150, vex::rotationUnits::deg);
  leftintake.spinFor(vex::directionType::fwd, 0.3, vex::rotationUnits::rev);

  // aim the center mogo
  sdrive.turnToHeading(45, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);

  // go to the central mogo
  goStraight(std::sqrt(2)*tileSize + 3, vex::directionType::fwd, 45, 90);  // fast
  sdrive.driveFor(2, vex::distanceUnits::in, 15, vex::velocityUnits::pct, false);
  vex::task::sleep(500); //wait for the slow walk to complete, but don't wait for too long!!!

  //grab the central mogo
  leftintake.spinFor(vex::directionType::rev, 0.35, vex::rotationUnits::rev, false);
  vex::task::sleep(500);

  lift.spinFor(vex::directionType::fwd, 150, vex::rotationUnits::deg);
  //sdrive.turnToHeading(0, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);
  //goStraight(30, vex::directionType::rev, 0, 50);
  sdrive.driveFor(vex::directionType::rev, 2 * tileSize, vex::distanceUnits::in, 85, vex::velocityUnits::pct, false);
}

void usercontrol( void ) {
  // int spin = 0;

  while (1) {

    if (rc.ButtonR1.pressing()) {
      lift.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonR2.pressing()) {
      lift.spin(vex::directionType::rev);
    }

    else {
      lift.stop(vex::brakeType::hold);
    }

    double leftMotorSpeed = rc.Axis3.position(vex::percentUnits::pct) * 0.7;
    double rightMotorSpeed = rc.Axis2.position(vex::percentUnits::pct) * 0.7;

    if (fabs(leftMotorSpeed) > 5.0) {
      backleftdrive.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      backleftdrive.spin(fwd);
      frontleftdrive.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      frontleftdrive.spin(fwd);
    }
    else {
      backleftdrive.stop(vex::brakeType::hold);
      frontleftdrive.stop(vex::brakeType::hold);
    }

    if (fabs(rightMotorSpeed) > 5.0) {
      backrightdrive.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      backrightdrive.spin(fwd);  
      frontrightdrive.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      frontrightdrive.spin(fwd);
    }
    else {
      backrightdrive.stop(vex::brakeType::hold);
      frontrightdrive.stop(vex::brakeType::hold);
    }

    if (rc.ButtonL2.pressing()) {
      leftintake.setVelocity(10, vex::percentUnits::pct);

      //intake.setVelocity(30, vex::percentUnits::pct);
      leftintake.spin(vex::directionType::fwd);
    }

    else if (rc.ButtonL1.pressing()) {
      
      leftintake.setVelocity(10, vex::percentUnits::pct);
      leftintake.spin(vex::directionType::rev);

    }

    else {
      leftintake.stop(vex::brakeType::hold);

    }

    if (rc.ButtonA.pressing()) {
      rightintake.setVelocity(10, vex::percentUnits::pct);
      rightintake.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonB.pressing()) {
      rightintake.setVelocity(10, vex::percentUnits::pct);
      rightintake.spin(vex::directionType::rev);
    }
    else {
      rightintake.stop(vex::brakeType::hold);
    }

    vex::task::sleep(50);
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Brain.Screen.print("Get ready to start!");
  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton();

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}