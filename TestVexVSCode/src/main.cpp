#include "vex.h"
#include <iostream>
#include <cmath>
#include <cassert>

using namespace vex;
using namespace std;

competition Competition;



void pre_auton(Roboto& robot) {
 
}

void autonomous( void ) {
  
}



void usercontrol( void ) {

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