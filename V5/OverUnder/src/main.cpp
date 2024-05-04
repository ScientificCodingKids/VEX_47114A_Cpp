#include "vex.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include "../../../Utils/nutils.hpp"
#include "../../../Utils/motion.hpp"

using namespace vex;
using namespace std;

competition Competition;


class RobotOverUnder: public DriveTrainBase {
  public:
    RobotOverUnder(vex::motor& bl, vex::motor& br, vex::motor& fl, vex::motor& fr, vex::brain& brn, vex::inertial& ins, vex::motor& cat)
    : DriveTrainBase(bl, br, fl, fr, brn, ins) {
      // TODO
    }

    void throw_obj() { /* TODO */ }

};  // class RobotOverUnder



RollingScreen rs(Brain.Screen);


void pre_auton(RobotOverUnder& robot) {
  rs.print("Enter pre_auton(): %d ", 0);

  //robot.calibrate();

}

void autonomous( void ) {

}

void usercontrol( void ) {
  bool isRunning = false;
  int currSpeed = 20;

  while (1) {
      if (rc.ButtonUp.pressing()) {
        if (currSpeed == 0) 
          cat.spin(vex::directionType::fwd);  // port 18
        else {
          currSpeed += 10;
          cat.setVelocity(currSpeed, velocityUnits::pct);
        }
          
        isRunning = true;
      }
      if (rc.ButtonDown.pressing()) {
        cat.stop(brakeType::coast);
        isRunning = false;
      }

      if (isRunning) {
        rs.print("%.0f | %.0f => v:%.0f, i:%.0f, p:%.0f \n", currSpeed, cat.velocity(), cat.voltage(), cat.current(), cat.power());
      }
      
      task::sleep(1000);
  }  // while
} // usercontrol




int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // our own code ONLY AFTER vexcodeInit()
  RobotOverUnder robot(backleftdrive, backrightdrive, frontleftdrive, frontrightdrive, Brain, inertialSensor, cat);
  robot.setRollingScreen(&rs);

  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton(robot);

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}