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
      vex::motor& aIntake): DriveTrainBase(bl, br, fl, fr, brn, ins), intake(aIntake) {
      // TODO
    }
  
    vex::motor& intake;
};  // class Roboto



RollingScreen rs(Brain.Screen);


void pre_auton(Roboto& robot) {
  rs.print("Enter pre_auton(): %d ", 0);

  //robot.calibrate();

}

void autonomous( void ) {

}

void usercontrol( void ) {
  bool isRunning = false;
  int currSpeed = 20;
  int maxSpeed = 90;

  vex::directionType intakeRotDir = vex::directionType::fwd;
  rs.print("START ... \n");

  while (1) {
      if (rc.ButtonA.pressing()) {
        intakeRotDir = vex::directionType::fwd;
      }

      if (rc.ButtonB.pressing()) {
        intakeRotDir = vex::directionType::rev;
      }
      intake.spin(intakeRotDir);

      if (rc.ButtonUp.pressing()) {
      
        currSpeed += 10;
        currSpeed = min(currSpeed, maxSpeed);
        intake.setVelocity(currSpeed, velocityUnits::pct);
        
        rs.print("SPIN at %.1f \n", currSpeed);

      }
      if (rc.ButtonDown.pressing()) {
        if (currSpeed == 0) {
          intake.stop(brakeType::coast);
          rs.print("STOP");
          //isRunning = false;
        }
        else {
          currSpeed -= 10;
          intake.setVelocity(currSpeed, velocityUnits::pct);
        }
      }

      // if (isRunning) {
      //   rs.print("%.0f | %.0f => v:%.0f, i:%.0f, p:%.0f \n", currSpeed, cat.velocity(), cat.voltage(), cat.current(), cat.power());
      // }
      
      task::sleep(1000);
  }  // while
} // usercontrol




int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // our own code ONLY AFTER vexcodeInit()
  Roboto robot(backleftdrive, backrightdrive, frontleftdrive, frontrightdrive, Brain, inertialSensor, intake);
  robot.setRollingScreen(&rs);

  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton(robot);

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}