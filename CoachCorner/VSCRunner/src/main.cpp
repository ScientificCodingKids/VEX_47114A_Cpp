/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\north                                            */
/*    Created:      Sun May 08 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

// This is a toy project to test out if we can
// 1) have intellisense in VSC
// 2) perform code compilation/link in VSC

#include "vex.h"
#include "nutils.hpp"
using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  auto ss = ScrollingScreen<int>(Brain);
}
