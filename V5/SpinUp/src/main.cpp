#include "vex.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include "../../../Utils/nutils.hpp"
#include "../../../Utils/motion.hpp"
#include "../../../Utils/XDrive.hpp"
#include <tuple>

using namespace vex;
using namespace std;

competition Competition;
XDriveRobot robot(backleftdrive, backrightdrive, frontleftdrive, frontrightdrive, Brain, inertialSensor, flywheel, indexer, intake);
RollingScreen rs(Brain.Screen);

double multby2(double x) {
  return x * 2.0;
}


void pre_auton( void ) {
  inertialSensor.calibrate();
  vex::task::sleep(2000);
}

void autonomous( void ) {
  autonWithXD(robot, 225.0, 50, 0., rs, 0.00);
}


void usercontrol( void ) {
  robot.calibrate();
  driveWithXD(robot, rc, rs, 0);
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