/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Chloe Ning                                       */
/*    Created:      Wed Jan 22 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}


void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  Brain.Screen.print("Hello, 47114A! Program to start. haha");
  //lift.stop(vex::brakeType::hold);
  claw.stop(vex::brakeType::hold);

}


void autonomous( void ) {
  // copied from our Robot Mesh code used in Roslyn event
    Brain.Screen.print("auto");

    // dt.driveFor(vex::directionType::fwd, 4, vex::distanceUnits::in);
    // dt.setDriveVelocity(25, vex::percentUnits::pct);
    // dt.driveFor(vex::directionType::fwd, 4, vex::distanceUnits::in);
    // claw.rotateFor(vex::directionType::rev, 4, vex::rotationUnits::rev);
    // dt.turnFor(vex::turnType::right, 130, vex::rotationUnits::deg);
    // dt.driveFor(vex::directionType::fwd, 8, vex::distanceUnits::in);
    // claw.rotateFor(vex::directionType::fwd, 2.5, vex::rotationUnits::rev);
    // dt.driveFor(vex::directionType::rev, 4, vex::distanceUnits::in);
}

void usercontrol( void ) {
  int pinch = 0;

  while (1) { // put all our code within this indefinite loop
  
    if (rc.ButtonUp.pressing()) {
      pinch = 1;
    }
    if (rc.ButtonR1.pressing()) {
      lift.spin(vex::directionType::fwd);
 //       leftdownlift.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonR2.pressing()) {
      lift.spin(vex::directionType::rev);
    //    leftdownlift.spin(vex::directionType::rev);
    }
    else {
      lift.stop(vex::brakeType::hold);
  //      leftdownlift.stop(vex::brakeType::hold);
    }
 
    double leftMotorSpeed = rc.Axis3.position(vex::percentUnits::pct) * 0.5;
    double rightMotorSpeed = rc.Axis2.position(vex::percentUnits::pct) * 0.5;
 
    double clawMoveSpeed = 20;
    double clawPinchSpeed = 0.5;  // force claw to pinch tightly

    if (fabs(leftMotorSpeed) > 5.0) {
      leftDriveMotor.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      leftDriveMotor.spin(fwd);
    }
    else {
      leftDriveMotor.stop(vex::brakeType::hold);
    }

    if (fabs(rightMotorSpeed) > 5.0) {
      rightDriveMotor.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      rightDriveMotor.spin(fwd);
    }
    else {
      rightDriveMotor.stop(vex::brakeType::hold);
    }


    if (rc.ButtonL1.pressing()) {
      claw.setVelocity(clawMoveSpeed, vex::percentUnits::pct);
      claw.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonL2.pressing()) {
      claw.setVelocity(clawMoveSpeed, vex::percentUnits::pct);
      claw.spin(vex::directionType::rev);
    }
    else {
      if (pinch == 1) {
        claw.setVelocity(clawPinchSpeed, vex::percentUnits::pct);
        claw.spin(vex::directionType::fwd);
      }
      else { 
        claw.stop(vex::brakeType::brake);
      }
    }

    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  }
}