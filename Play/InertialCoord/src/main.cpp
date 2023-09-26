/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Chloe Ning                                       */
/*    Created:      Wed Jan 22 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------xsssssssssss------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    10, 16, 1, 15   
// lift                 motor_group   4, 9            
// mogo1                motor         18              
// mogo2                motor         20              
// ---- END VEXCODE CONFIGURED DEVICES ----

//#include "C:/Program Files (x86)/VEX Robotics/VEXcode Pro V5/sdk/vexv5/include/vex_units.h"
#include "vex.h"
#include <iostream>
#include <cmath>
#include "../../../Utils/motion.hpp"
#include <cassert>
#include "../../../Utils/nutils.hpp"

using namespace vex;
using namespace std;

competition Competition;


void pre_auton( void ) {
}

void autonomous( void ) {

}

void usercontrol( void ) {
  
} // usercontrol

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  

  vex::task::sleep(2500);

  // Coord actualDestLoc = makeTurn(90, true);
  // SmartScreen ss(Brain.Screen, 3, 6);
  // ss.printAt(1, "actual loc: (%.2f, %.2f)",actualDestLoc.x, actualDestLoc.y);
  // vex::task::sleep(3000);
  // ss.printAt(2, "actual loc after three seconds: (%.2f, %.2f)", actualDestLoc.x, actualDestLoc.y);

  goPlatform();
  Brain.Screen.print("done");

  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton();

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}