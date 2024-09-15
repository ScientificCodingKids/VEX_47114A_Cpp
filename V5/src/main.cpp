#include "vex.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include "nutils.hpp"
#include "motion.hpp"

using namespace vex;
using namespace std;

competition Competition;


class Roboto: public DriveTrainBase {
  public:
    Roboto(vex::motor& bl, vex::motor& br, vex::motor& fl, vex::motor& fr, 
      vex::brain& brn, vex::inertial& ins, 
      vex::motor& aIntake, vex::motor& convey, RollingScreen rs): DriveTrainBase(bl, br, fl, fr, brn, ins, rs), intake(aIntake), conveyor(convey) {
      // TODO
    }
  
    vex::motor& intake;
    vex::motor& conveyor;
};  // class Roboto



RollingScreen rs(Brain.Screen);

Roboto robot(backleftdrive, backrightdrive, frontleftdrive, frontrightdrive, Brain, inertialSensor, intake, conveyor, rs);


void pre_auton(Roboto& robot) {
 
  robot.screen.print("Enter pre_auton(): %d \n", 0);

  robot.calibrate();
  
  robot.screen.print("1: calib done %p \n", (void*)(&rs));

}

void autonomous( void ) {
  // go straight and suck in
  //frontleftdrive.spin(directionType::fwd);
  int v0 = 0;

  robot.screen.print("$Start auton %d \n", v0);

  robot.goStraight(5., directionType::fwd, 0., 80);
  
  robot.screen.print("!BBBB goStraightDone %d \n", v0);

  robot.conveyor.setVelocity(70.0, velocityUnits::pct);
  robot.intake.setVelocity(70., velocityUnits::pct);
  robot.intake.spin(directionType::fwd);
  robot.conveyor.spin(directionType::fwd);

  robot.goStraight(3., directionType::fwd, 0., 10.);

  robot.makeTurn(90., true, 25.);
  
}


void usercontrol( void ) {
  // remote control key mapping
  // https://kb.vex.com/hc/en-us/articles/360035954651-Using-Blocks-for-Controller-Buttons-Joysticks-in-VEXcode-V5


  int currSpeed = 20;
  int maxSpeed = 90;

  int leftJoyPos;
  int rightJoyPos;

  int leftDriveSpeed;
  int rightDriveSpeed;

  directionType leftDriveDir = directionType::fwd;
  directionType rightDriveDir = directionType::fwd;


  vex::directionType intakeRotDir = vex::directionType::fwd;
  rs.print("START ... \n");

  while (1) {
    leftJoyPos = rc.Axis3.position();
    rightJoyPos = rc.Axis2.position();
    leftDriveSpeed = abs(leftJoyPos * 0.85);
    rightDriveSpeed = abs(rightJoyPos * 0.85);

    if (leftJoyPos > 5) {
      leftDriveDir = directionType::fwd;
      frontleftdrive.setVelocity(leftDriveSpeed, velocityUnits::pct);
      backleftdrive.setVelocity(leftDriveSpeed, velocityUnits::pct);
    }
    else if (leftJoyPos < -5) {
      leftDriveDir = directionType::rev;
      frontleftdrive.setVelocity(leftDriveSpeed, velocityUnits::pct);
      backleftdrive.setVelocity(leftDriveSpeed, velocityUnits::pct);
    }
    else {
      frontleftdrive.setVelocity(0, velocityUnits::pct);
      backleftdrive.setVelocity(0, velocityUnits::pct);
    }

    if (rightJoyPos > 5) {
      rightDriveDir = directionType::fwd;
      frontrightdrive.setVelocity(rightDriveSpeed, velocityUnits::pct);
      backrightdrive.setVelocity(rightDriveSpeed, velocityUnits::pct);
    }
    else if (rightJoyPos < -5) {
      rightDriveDir = directionType::rev;
      frontrightdrive.setVelocity(rightDriveSpeed, velocityUnits::pct);
      backrightdrive.setVelocity(rightDriveSpeed, velocityUnits::pct);
    }
    else {
      frontrightdrive.setVelocity(0, velocityUnits::pct);
      backrightdrive.setVelocity(0, velocityUnits::pct);
    }

    frontleftdrive.spin(leftDriveDir);
    frontrightdrive.spin(rightDriveDir);
    backleftdrive.spin(leftDriveDir);
    backrightdrive.spin(rightDriveDir);


    if (rc.ButtonR1.pressing()) {
      currSpeed = min(currSpeed + 10, maxSpeed);
    }

    if (rc.ButtonR2.pressing()) {
      currSpeed = max(currSpeed - 10, 0);
    }

    intake.setVelocity(currSpeed, velocityUnits::pct);
    conveyor.setVelocity(currSpeed, velocityUnits::pct);
    rs.print("Intake SPIN at %.1f \n", currSpeed);
    
    if (rc.ButtonUp.pressing()) {
      intake.spin(directionType::fwd);
    }
    else if (rc.ButtonDown.pressing()) {
      intake.spin(directionType::rev);
    }
    else if (rc.ButtonX.pressing()) {
      intake.stop(brakeType::coast);
    }

    if (rc.ButtonLeft.pressing()) {
      conveyor.spin(directionType::fwd);
    }
    else if (rc.ButtonRight.pressing()) {
      conveyor.spin(directionType::rev);
    }
    else if (rc.ButtonY.pressing()) {
      conveyor.stop(brakeType::coast);
    }

    if (rc.ButtonL1.pressing()) {
      mogomech.set(true);
    }
    else if (rc.ButtonL2.pressing()) {
      mogomech.set(false);
    }
  
    task::sleep(100);
  }  // while
} // usercontrol




int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // our own code ONLY AFTER vexcodeInit()

  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton(robot);

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}