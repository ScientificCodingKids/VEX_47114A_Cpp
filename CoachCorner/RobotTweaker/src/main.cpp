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
using namespace std;
using namespace vex;

competition Competition;

auto ss = ScrollingScreen<int>();

void pre_auton( void ) {
}


void autonomous( void ) {
 
  }


// use L and R buttons to drive each wheel independently
// use E and F buttons to op each lift arm independently
// use left and right joysticks to adjust speed

void usercontrol( void ) {
  double wheelSpeed = 50;
  double liftSpeed = 30;

  double intakeSpeed = 50;  // fixed speed

  while (1) {
    
    wheelSpeed *= 1.0 + rc.Axis3.position(vex::percentUnits::pct)/100.0 * 0.01;
    liftSpeed *= 1.0 + rc.Axis2.position(vex::percentUnits::pct)/100.0 * 0.05;

    wheelSpeed = max(min(wheelSpeed, 80.), 10.0);
    liftSpeed = max(min(liftSpeed, 90.0), 20.0);

    frontleftdrive.setVelocity(wheelSpeed, vex::velocityUnits::pct);
    frontrightdrive.setVelocity(wheelSpeed, vex::velocityUnits::pct);
    backleftdrive.setVelocity(wheelSpeed, vex::velocityUnits::pct);
    backrightdrive.setVelocity(wheelSpeed, vex::velocityUnits::pct);

    leftlift.setVelocity(liftSpeed, vex::velocityUnits::pct);
    rightlift.setVelocity(liftSpeed, vex::velocityUnits::pct);

    // drive train motion
    if (rc.ButtonL1.pressing()) {
      frontleftdrive.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonL2.pressing()) {
      backleftdrive.spin(vex::directionType::fwd);
    }

    if (rc.ButtonR1.pressing()) {
      frontrightdrive.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonR2.pressing()) {
      backrightdrive.spin(vex::directionType::fwd);
    }

    // lift motion
    if (rc.ButtonUp.pressing()) {
      leftlift.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonDown.pressing()) {
      leftlift.spin(vex::directionType::rev);
    }
    else {
      leftlift.setBrake(vex::brakeType::hold);
    }

    if (rc.ButtonX.pressing()) {
      rightlift.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonB.pressing()) {
      rightlift.spin(vex::directionType::rev);
    }
    else {
      rightlift.setBrake(vex::brakeType::hold);
    }

    // intake motion
    if (rc.ButtonRight.pressing()) {
      leftintake.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonLeft.pressing()) {
      leftintake.spin(vex::directionType::rev);
    }
    else {
      leftintake.setBrake(vex::brakeType::coast);
    }

    if (rc.ButtonA.pressing()) {
      rightintake.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonY.pressing()) {
      rightintake.spin(vex::directionType::rev);
    }
    else {
      rightintake.setBrake(vex::brakeType::coast);
    }

    ss.print("[Speed] wheel: %4.1f, lift: %4.1f, intake: %4.1f \n", wheelSpeed, liftSpeed, intakeSpeed);

    vex::task::sleep(100);
  }  // while
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