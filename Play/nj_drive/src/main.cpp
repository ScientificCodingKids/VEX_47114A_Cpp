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


using namespace vex;
competition Competition;

// use heading(), data range [0, 360]. so calibrate to 0 means the initial reading can be 365.5 !

void pre_auton ( void ) {
  
}

void autonomous ( void ) {

}

void usercontrol ( void ) {
  auto liftspeed = 20;
      
  lift.setVelocity(liftspeed, vex::velocityUnits::pct);
  
  while (1) {
    double leftMotorSpeed = rc.Axis3.position(vex::percentUnits::pct) * 0.5;
    double rightMotorSpeed = rc.Axis2.position(vex::percentUnits::pct) * 0.5;

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

    if (rc.ButtonR1.pressing()) {
      lift.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonR2.pressing()) {
      lift.spin(vex::directionType::rev);
    }

    else {
      lift.stop(vex::brakeType::hold);
    }

    vex::task::sleep(50);
  } // while loop

} // usercontrol

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