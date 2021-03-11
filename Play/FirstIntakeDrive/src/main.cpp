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

using namespace vex;

competition Competition;

auto ss = ScrollingScreen<int>();

void pre_auton( void ) {
}


void autonomous( void ) {
 
  }

void usercontrol( void ) {
  int spin = 0;

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

    if (rc.ButtonL2.pressing()) {
      // leftintake.setVelocity(30, vex::percentUnits::pct);
      // rightintake.setVelocity(30, vex::percentUnits::pct);

      // //intake.setVelocity(30, vex::percentUnits::pct);
      // leftintake.spin(vex::directionType::fwd);
      // rightintake.spin(vex::directionType::fwd);
      if (spin == 0 or spin == -1) {
        // aiden is so annoying.
        spin = 1;
        leftintake.setVelocity(70, vex::percentUnits::pct);
        rightintake.setVelocity(70, vex::percentUnits::pct);

        leftintake.spin(vex::directionType::fwd);
        rightintake.spin(vex::directionType::fwd);
      }

      else if (spin == 1) {
        spin = 0;
        leftintake.stop(vex::brakeType::hold);
        rightintake.stop(vex::brakeType::hold);
      }
    }

    else if (rc.ButtonL1.pressing()) {
      //intake.setVelocity(30, vex::percentUnits::pct);
      //intake.spin(vex::directionType::rev);
      
    //   leftintake.setVelocity(30, vex::percentUnits::pct);
    //   rightintake.setVelocity(30, vex::percentUnits::pct);

    //   //intake.setVelocity(30, vex::percentUnits::pct);
    //   leftintake.spin(vex::directionType::rev);
    //   rightintake.spin(vex::directionType::rev);
    // }

    // else {
    //   leftintake.stop(vex::brakeType::hold);
    //   rightintake.stop(vex::brakeType::hold);
    
    if (spin == 0 or spin == 1) {
        spin = -1;
        leftintake.setVelocity(70, vex::percentUnits::pct);
        rightintake.setVelocity(70, vex::percentUnits::pct);

        leftintake.spin(vex::directionType::rev);
        rightintake.spin(vex::directionType::rev);
      }

      else if (spin == -1) {
        spin = 0;
        leftintake.stop(vex::brakeType::hold);
        rightintake.stop(vex::brakeType::hold);
      }

    }

    // else {
    //   intake.stop(vex::brakeType::hold);
    // }

    ss.print("Speed: %4.1f, %4.1f \n", leftMotorSpeed, rightMotorSpeed);

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