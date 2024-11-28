#include "vex.h"

using namespace vex;


// A global instance of competition
competition Competition;

vex::motor intake = vex::motor(PORT5, true);

vex::brain Brain;

vex::controller rc;

vex::motor backleftdrive = vex::motor(PORT3, true);
vex::motor backrightdrive = vex::motor(PORT4);
vex::motor frontleftdrive = vex::motor(PORT1, true);
vex::motor frontrightdrive = vex::motor(PORT2);

motor_group leftDrive = motor_group(backleftdrive, frontleftdrive);
motor_group rightDrive = motor_group(backrightdrive, frontrightdrive);

bumper bumpBase(Brain.ThreeWirePort.D);
bumper bumpBase2(Brain.ThreeWirePort.E);

pneumatics mogomech(Brain.ThreeWirePort.F);
vex::inertial inertialSensor = vex::inertial(PORT15);
vex::motor conveyor = vex::motor(PORT7);
