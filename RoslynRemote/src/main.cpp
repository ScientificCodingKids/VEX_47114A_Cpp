/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Chloe Ning                                       */
/*    Created:      Wed Dec 25 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "robot-config.h"

using namespace vex;

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  Brain.Screen.print("Hello, 47114A! Program to start.");
  lift.stop(vex::brakeType::hold);
  Brain.Screen.print("Temp: %3.1f, %3.1f", leftdownlift.temperature(vex::temperatureUnits::celsius), rightdownlift.temperature(vex::temperatureUnits::celsius));
}


void autonomous( void ) {
  // copied from our Robot Mesh code used in Roslyn event
    dt.setDriveVelocity(100, vex::velocityUnits::pct);
    dt.driveFor(vex::directionType::rev, 50, vex::distanceUnits::cm);
    dt.driveFor(vex::directionType::fwd, 15, vex::distanceUnits::cm);
}

void usercontrol( void ) {
  while (1) { // put all our code within this indefinite loop
    if (rc.ButtonR1.pressing()) {
      lift.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonR2.pressing()) {
      lift.spin(vex::directionType::rev);
    }
    else {
      lift.stop(vex::brakeType::hold);
    }
 
    double leftMotorSpeed = rc.Axis3.position(vex::percentUnits::pct) * 0.5;
    double rightMotorSpeed = rc.Axis2.position(vex::percentUnits::pct) * 0.5;

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

    // if (rc.ButtonL1.pressing()) {
    //   liftLeft.spin(vex::directionType::fwd);
    // }
    // else if (rc.ButtonL2.pressing()) {
    //   liftRight.spin(vex::directionType::fwd);
    // }
    // else {
    //   liftLeft.stop(vex::brakeType::hold);
    //   liftRight.stop(vex::brakeType::hold);
    // }


    // if (rc.ButtonL1.pressing()) {
    //   claw.spin(vex::directionType::fwd);
    // }
    // else if (rc.ButtonL2.pressing()) {
    //   claw.spin(vex::directionType::rev);
    // }
    // else {
    //   claw.stop(vex::brakeType::hold);
    // }

    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  }
}

competition Competition;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
  //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    
    //Run the pre-autonomous function. 
    pre_auton();
       
    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    

    return 0;
}
