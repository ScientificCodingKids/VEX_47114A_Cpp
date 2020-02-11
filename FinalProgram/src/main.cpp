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

competition Competition;



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
    lift.spinFor(vex::directionType::fwd, 100, vex::rotationUnits::deg);
    dt.setDriveVelocity(75, vex::percentUnits::pct);
    dt.driveFor(vex::directionType::rev, 10, vex::distanceUnits::in);
    dt.turnFor(vex::turnType::right, 110, vex::rotationUnits::deg);
    dt.setDriveVelocity(25, vex::percentUnits::pct);
    dt.driveFor(vex::directionType::fwd, 4.5, vex::distanceUnits::in);
    dt.drive(vex::directionType::fwd, 25, vex::velocityUnits::pct);
    claw.rotateFor(vex::directionType::fwd, 120, vex::rotationUnits::deg);
    dt.stop();
}

void usercontrol( void ) {
  int pinch = 0;

  while (1) { // put all our code within this indefinite loop
  
    if (rc.ButtonUp.pressing()) {
      pinch = 1;
    }
    if (rc.ButtonR1.pressing()) {
      lift.spin(vex::directionType::fwd);
      //  rightdownlift.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonR2.pressing()) {
      lift.spin(vex::directionType::rev);
      //  rightdownlift.spin(vex::directionType::rev);
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