#include "vex.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include "../../../Utils/nutils.hpp"
#include "../../../Utils/motion.hpp"

using namespace vex;
using namespace std;

competition Competition;


void pre_auton( void ) {
  inertialSensor.calibrate();
  vex::task::sleep(2000);
}

void autonomous( void ) {
  
}

void usercontrol( void ) {
  double driveMode = 0;
  double driveSpeed = 0;
  double flspeed = 0;
  double frspeed = 0;
  double blspeed = 0;
  double brspeed = 0;
  bool flfwd = true;
  bool frfwd = true;
  bool blfwd = true;
  bool brfwd = true;
  double arc = 0;
  double angle = 0;
  // 0 = stationary
  // 1 = forwards
  // 2 = backwards
  // 3 = left
  // 4 = right
  // 5 = strafe

  
  while (1) {
    double x = rc.Axis4.position(vex::percentUnits::pct);
    double y = rc.Axis3.position(vex::percentUnits::pct);
    arc = asin(abs(y)/sqrt(x*x + y*y));
    if (x>=0) {
      if (y>=0) arc = pi/2 - arc;
      else arc = 3pi/2 + arc;
    }
    else {
      if (y>=0) arc = pi/2 + arc;
      else arc = 3pi/2 - arc;
    }
    angle = arc2deg(arc);
    
    


    if (rc.Axis3.position(vex::percentUnits::pct) > 5) {
      driveSpeed = rc.Axis3.position(vex::percentUnits::pct) * 0.5;
      driveMode = 1;
      frontleftdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      frontleftdrive.spin(vex::directionType::fwd);
      backleftdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      backleftdrive.spin(directionType::fwd);
      frontrightdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      frontrightdrive.spin(directionType::fwd);
      backrightdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      backrightdrive.spin(directionType::fwd);
    }

    if (rc.Axis3.position(vex::percentUnits::pct) < 5) {
      driveSpeed = abs(rc.Axis3.position(vex::percentUnits::pct)) * 0.5;
      driveMode = 2;
      frontleftdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      frontleftdrive.spin(directionType::rev);
      backleftdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      backleftdrive.spin(directionType::rev);
      frontrightdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      frontrightdrive.spin(directionType::rev);
      backrightdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      backrightdrive.spin(directionType::rev);
    }

    else if (rc.Axis4.position(vex::percentUnits::pct) > 5) {
      driveSpeed = rc.Axis4.position(vex::percentUnits::pct) * 0.5;
      driveMode = 3;
      frontleftdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      frontleftdrive.spin(directionType::rev);
      backleftdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      backleftdrive.spin(directionType::fwd);
      frontrightdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      frontrightdrive.spin(directionType::rev);
      backrightdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      backrightdrive.spin(directionType::fwd);
    }

    else if (rc.Axis4.position(vex::percentUnits::pct) < 5) {
      driveSpeed = abs(rc.Axis4.position(vex::percentUnits::pct)) * 0.5;
      driveMode = 4;
      frontleftdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      frontleftdrive.spin(directionType::fwd);
      backleftdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      backleftdrive.spin(directionType::rev);
      frontrightdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      frontrightdrive.spin(directionType::fwd);
      backrightdrive.setVelocity(driveSpeed, vex::velocityUnits::pct);
      backrightdrive.spin(directionType::rev);
     }

    vex::task::sleep(10);
  }
  
} // usercontrol

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton();

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}