#include "vex.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include "../../../Utils/nutils.hpp"
#include "../../../Utils/motion.hpp"

using namespace vex;
using namespace std;

competition Competition;

double multby2(double x) {
  return x * 2.0;
}


void pre_auton( void ) {
  inertialSensor.calibrate();
  vex::task::sleep(2000);
}

void autonomous( void ) {

}

void usercontrol( void ) {
  
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