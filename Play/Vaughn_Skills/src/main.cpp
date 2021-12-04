/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Chloe Ning                                       */
/*    Created:      Wed Jan 22 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------xsssssssssss------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    10, 16, 1, 15   
// lift                 motor_group   4, 9            
// mogo1                motor         18              
// mogo2                motor         20              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "../../../Utils/nutils.hpp"

using namespace vex;

competition Competition;

// auto ss = ScrollingScreen<int>();

void pre_auton( void ) {
  //inertial_sensor.calibrate();
}


void autonomous( void ) {
  inertial_sensor.calibrate();
  vex::task::sleep(2000); 

  lift.rotateFor(vex::directionType::rev, 50, vex::rotationUnits::deg);

  for (int i = 0; i< 7; ++i) {
    sdrive.driveFor(vex::directionType::fwd, 3.5 * 24 / 7 + 0.5, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
    sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);

  }
  // sdrive.driveFor(vex::directionType::fwd, 3.5 * 24, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  sdrive.driveFor(vex::directionType::rev, 2 * 24, vex::distanceUnits::in, 60, vex::velocityUnits::pct);

  vex::task::sleep(500);
  sdrive.turnToHeading(270, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  vex::task::sleep(500);

  sdrive.driveFor(vex::directionType::fwd, 24, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
    
  vex::task::sleep(500);
  sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  vex::task::sleep(500);

  
  // push neutral mobile goal 1
  sdrive.driveFor(vex::directionType::fwd, 2.5 * 24, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  sdrive.driveFor(vex::directionType::rev, 2 * 24, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  
  vex::task::sleep(500);
  sdrive.turnToHeading(270, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  vex::task::sleep(500);

  sdrive.driveFor(vex::directionType::fwd, 36, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);

  sdrive.driveFor(vex::directionType::fwd, 2 * 24, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  sdrive.driveFor(vex::directionType::rev, 2 * 24, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  sdrive.turnToHeading(270, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  sdrive.driveFor(vex::directionType::fwd, 36, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);

  sdrive.driveFor(vex::directionType::fwd, 2 * 24, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  sdrive.driveFor(vex::directionType::rev, 24, vex::distanceUnits::in);


  // vex::task::sleep(2000);
  // // alliance mobile goal
  // sdrive.driveFor(vex::directionType::rev, 8, vex::distanceUnits::in, 60, vex::velocityUnits::pct);

  // for (int i = 0; i< 8; ++i) {
  //   sdrive.driveFor(vex::directionType::rev, 9 + 1, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  //   sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);

  // }
  // sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);

  // vex::task::sleep(2000);

  // sdrive.driveFor(vex::directionType::fwd, 43, vex::distanceUnits::in, 70, vex::velocityUnits::pct);
  // sdrive.turnToHeading(270, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  // sdrive.driveFor(vex::directionType::rev, 24, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  // sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  // // neutral 1
  // sdrive.driveFor(vex::directionType::rev, 55, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  // sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  // sdrive.driveFor(vex::directionType::fwd, 40, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  // sdrive.turnToHeading(270, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  // sdrive.driveFor(vex::directionType::rev, 36, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  // sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  // // neutral 2
  // sdrive.driveFor(vex::directionType::rev, 30, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  // sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  // sdrive.driveFor(vex::directionType::rev, 30, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  // sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  // sdrive.driveFor(vex::directionType::fwd, 45, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  // sdrive.turnToHeading(270, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  // sdrive.driveFor(vex::directionType::rev, 33, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  // sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  // // neutral 3
  // sdrive.driveFor(vex::directionType::rev, 60, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  // sdrive.driveFor(vex::directionType::fwd, 24, vex::distanceUnits::in, 60, vex::velocityUnits::pct);
  // // grabbing
  // lift.rotateFor(vex::directionType::rev, 70, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  // sdrive.turnToHeading(150, vex::rotationUnits::deg);
  // sdrive.driveFor(vex::directionType::fwd, 1.4 * 24, vex::distanceUnits::in, 30, vex::velocityUnits::pct);
  // leftintake.rotateFor(vex::directionType::rev, 0.3, vex::rotationUnits::rev);
  // sdrive.driveFor(vex::directionType::rev, 10, vex::distanceUnits::in);
  }

  // before, 45 then 17

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