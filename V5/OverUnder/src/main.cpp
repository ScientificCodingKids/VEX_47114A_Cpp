#include "vex.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include "../include/robot-config.h"
#include "../../../Utils/nutils.hpp"
#include "../../../Utils/motion.hpp"

using namespace vex;
using namespace std;

competition Competition;


class RobotOverUnder: public DriveTrainBase {
  public:
    RobotOverUnder(vex::motor& bl, vex::motor& br, vex::motor& fl, vex::motor& fr, vex::brain& brn, vex::inertial& ins, vex::motor& cat, vex::motor&inta, vex::motor&intl)
    : DriveTrainBase(bl, br, fl, fr, brn, ins), catMotor(cat), intakeMotor(inta), intakeLifter(intl) {
    
    }

    vex::motor& catMotor;
    vex::motor& intakeMotor;
    vex::motor& intakeLifter;

    void throw_obj() { /* TODO */ }

};  // class RobotOverUnder



RollingScreen rs(theBrain.Screen);

RobotOverUnder theRobot(backleftdrive, backrightdrive, frontleftdrive, frontrightdrive, theBrain, inertialSensor, cat, intakeMotor, intakeLifter);

void experiment_catapult(RobotOverUnder& robot, vex::controller& rc) {
  const controller::button& btnSlipRotCW = rc.ButtonLeft;
  const controller::button& btnSlipRotCCW = rc.ButtonRight;

  const controller::button& btnModeSwitch = rc.ButtonL1;
  int catState = 0;  // 0: no spin; 1: spin

  const controller::button& btnSpeedUp = rc.ButtonR1;
  const controller::button& btnSpeedDn = rc.ButtonR2;

  double c = 60.0 / 36.0; // gear ratio

  double speed = 80.;

  robot.intakeLifter.setBrake(brakeType::coast);

  while (1) {

    double leftMotorSpeed = rc.Axis3.position(vex::percentUnits::pct) * 0.85;
    double rightMotorSpeed = rc.Axis2.position(vex::percentUnits::pct) * 0.85;

    if (fabs(leftMotorSpeed) > 5.0) {
      backleftdrive.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      backleftdrive.spin(fwd);
      frontleftdrive.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      frontleftdrive.spin(fwd);
    }
    else {
      backleftdrive.stop();
      frontleftdrive.stop();
    }

    if (fabs(rightMotorSpeed) > 5.0) {
      backrightdrive.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      backrightdrive.spin(fwd);  
      frontrightdrive.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      frontrightdrive.spin(fwd);
    }
    else {
      backrightdrive.stop();
      frontrightdrive.stop();
    }

    double oldSpeed = speed;

    if (btnSpeedUp.pressing()) {
      speed += 10.;
    }
    if (btnSpeedDn.pressing()) {
      speed -= 10.;
    }

    speed = min(80., max(10., speed));

    if (oldSpeed != speed) {
      rs.print("curr speed: %.1f", speed);
    }

    if (btnSlipRotCW.pressing()) {
      robot.catMotor.spin(directionType::fwd, speed, velocityUnits::pct);
    }
    else if (btnSlipRotCCW.pressing()) {
      robot.catMotor.spin(directionType::rev, speed, velocityUnits::pct);
    }
    else if (btnModeSwitch.pressing()) {
      robot.catMotor.stop(brakeType::hold);
    }

    if (rc.ButtonL2.pressing()) {
      robot.intakeMotor.spin(directionType::fwd, 50, velocityUnits::pct);
    }
    else if (rc.ButtonY.pressing()) {
      robot.intakeMotor.spin(directionType::rev, 50, velocityUnits::pct);
    }
    else if (rc.ButtonX.pressing()) {
      robot.intakeMotor.stop(brakeType::coast);
    }

    if (rc.ButtonUp.pressing()) {
      robot.intakeLifter.spin(directionType::fwd);
      robot.intakeLifter.setBrake(brakeType::coast);
    }
    else if (rc.ButtonDown.pressing()) {
      robot.intakeLifter.spin(directionType::rev);
      robot.intakeLifter.setBrake(brakeType::hold);
    }
    else {
      robot.intakeLifter.stop();
    }

    task::sleep(100);
  }
}


void pre_auton(RobotOverUnder& robot) {
  rs.print("Enter pre_auton(): %d ", 0);

  robot.calibrate();
  rs.print("Hello im done: %d", 1);

}

void straightBackAuton(RobotOverUnder& robot) {
  robot.goStraight(70, directionType::rev, 0, 70);
  robot.goStraight(10, directionType::fwd, 0, 70);
  robot.makeTurn(330, false);
  robot.goStraight(20, directionType::rev, 330, 70);

}

void skillsAuton(RobotOverUnder& robot) {
  robot.goStraight(3, directionType::fwd, 0, 50);
  robot.makeTurn(330, false, 30);
  robot.goStraight(5, directionType::fwd, 330, 30);

  robot.catMotor.spin(directionType::fwd, 85, velocityUnits::pct);
 
  bool isDone = false;
  int counter = 0;

  int time1 = 410;
  int time2 = 10;

  while (!isDone) {
    if (counter == time1) {isDone = true;}
    if (rc.ButtonUp.pressing()) {isDone = true;}
    counter += 1;
    task::sleep(100);
  }

  robot.catMotor.stop(brakeType::hold);

  // going over
  robot.makeTurn(12, true, 50);
  robot.goStraight(13, directionType::rev, 10, 50);
  robot.goStraight(108, directionType::rev,355, 80);

  // first push from the side
  robot.makeTurn(300, false, 50);
  robot.goStraight(50, directionType::rev, 300, 80);

  // positionig for second
  robot.goStraight(24, directionType::fwd, 300, 80);
  robot.makeTurn(210, false, 50);
  robot.goStraight(45, directionType::rev, 210, 80);

  robot.makeTurn(310, true, 50);
  robot.goStraight(65, directionType::rev, 310, 80);

}

void autonomous( void ) {

  skillsAuton(theRobot);

  
}

void usercontrol( void ) {
  experiment_catapult(theRobot, rc);
} // usercontrol




int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // our own code ONLY AFTER vexcodeInit()
  RobotOverUnder robot(backleftdrive, backrightdrive, frontleftdrive, frontrightdrive, theBrain, inertialSensor, cat, intakeMotor, intakeLifter);
  robot.setRollingScreen(&rs);

  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton(robot);

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}