/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       {author}                                                  */
/*    Created:      {date}                                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include <cstdlib>

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;
// A global instance of vex::competition
vex::competition Competition;

// define your global instances of motors and other devices here
vex::motor leftDriveMotor = vex::motor(vex::PORT10);
vex::motor rightDriveMotor = vex::motor(vex::PORT1, true);

vex::drivetrain dt = vex::drivetrain(leftDriveMotor, rightDriveMotor);

vex::motor rightuplift = vex::motor(vex::PORT4);
vex::motor rightdownlift = vex::motor(vex::PORT7, true);
vex::motor leftuplift = vex::motor(vex::PORT6, true);
vex::motor leftdownlift = vex::motor(vex::PORT5);

vex::motor_group lift = vex::motor_group(rightuplift, rightdownlift, leftdownlift, leftuplift);

vex::motor claw = vex::motor(vex::PORT20);

vex::controller rc = vex::controller();

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  Brain.Screen.print("Hello, 47114A! Program to start.");
}


void autonomous( void ) {
  // copied from our Robot Mesh code used in Roslyn event
    dt.setVelocity(100, vex::velocityUnits::pct);
    dt.driveFor(vex::directionType::rev, 50, vex::distanceUnits::cm);
    dt.driveFor(vex::directionType::fwd, 15, vex::distanceUnits::cm);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

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
    }
    else {
      leftDriveMotor.stop(vex::brakeType::hold);
    }

    if (fabs(rightMotorSpeed) > 5.0) {
      rightDriveMotor.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
    }
    else {
      rightDriveMotor.stop(vex::brakeType::hold);
    }

    if (rc.ButtonL1.pressing()) {
      claw.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonL2.pressing()) {
      claw.spin(vex::directionType::rev);
    }
    else {
      claw.stop(vex::brakeType::hold);
    }

    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
    //Set up callbacks for autonomous and driver control periods.
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    
    //Run the pre-autonomous function. 
    pre_auton();
       
    //Prevent main from exiting with an infinite loop.                        
    while(1) {
      vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
    }    
}