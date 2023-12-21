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
    RobotOverUnder(vex::motor& bl, vex::motor& br, vex::motor& fl, vex::motor& fr, vex::brain& brn, vex::inertial& ins, vex::motor& cat)
    : DriveTrainBase(bl, br, fl, fr, brn, ins), catMotor(cat) {
    
    }

    vex::motor& catMotor;

    void throw_obj() { /* TODO */ }

};  // class RobotOverUnder



RollingScreen rs(theBrain.Screen);

RobotOverUnder theRobot(backleftdrive, backrightdrive, frontleftdrive, frontrightdrive, theBrain, inertialSensor, cat);


void experiment_catapult(RobotOverUnder& robot, vex::controller& rc) {
  const controller::button& btnSlipRotCW = rc.ButtonLeft;
  const controller::button& btnSlipRotCCW = rc.ButtonRight;
  //controller::button& btnThrow = rc.ButtonUp;

  const controller::button& btnModeSwitch = rc.ButtonL1;
  int catState = 0;  // 0: no spin; 1: spin


  const controller::button& btnSpeedUp = rc.ButtonR1;
  const controller::button& btnSpeedDn = rc.ButtonR2;


  double c = 60.0 / 36.0; // gear ratio

  double theta_0 = 60.0 / 360. * 2.0 * M_PI;  // starting angle
  double theta_s = -30.0 / 360. * 2.0 * M_PI; // shooting angle

  double speed = 20.;

  while (1) {
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

    int oldState = catState;

    if (btnModeSwitch.pressing()) {
      catState = 1 - catState;
    }

    if (oldState != catState) {
      rs.print("Curr state: %d", catState);
    }

    if (btnSlipRotCW.pressing()) {
      robot.catMotor.spin(directionType::fwd, speed, velocityUnits::pct);
    }

    if (btnSlipRotCCW.pressing()) {
      robot.catMotor.spin(directionType::rev, speed, velocityUnits::pct);
    }


    task::sleep(100);
  }


}


void pre_auton(RobotOverUnder& robot) {
  rs.print("Enter pre_auton(): %d ", 0);

  robot.calibrate();

}

void autonomous( void ) {

}

void usercontrol( void ) {
  experiment_catapult(theRobot, rc);
} // usercontrol




int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // our own code ONLY AFTER vexcodeInit()
  RobotOverUnder robot(backleftdrive, backrightdrive, frontleftdrive, frontrightdrive, theBrain, inertialSensor, cat);
  robot.setRollingScreen(&rs);

  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton(robot);

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}