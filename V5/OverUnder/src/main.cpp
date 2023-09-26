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

  robot.calibrate();

}

void autonomous( void ) {

}

void usercontrol( void ) {
  
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