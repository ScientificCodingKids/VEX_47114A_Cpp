#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

vex::motor backleftdrive = vex::motor(PORT3, true);
vex::motor backrightdrive = vex::motor(PORT4);
vex::motor frontleftdrive = vex::motor(PORT1);
vex::motor frontrightdrive = vex::motor(PORT2, true);

vex::motor_group leftdrive = vex::motor_group(backleftdrive, frontleftdrive);
vex::motor_group rightdrive = vex::motor_group(backrightdrive, frontrightdrive);

vex::inertial inertialSensor = vex::inertial(PORT8);

vex::drivetrain dt = vex::drivetrain(leftdrive, rightdrive, 320, 260, 280);
vex::smartdrive sdrive = vex::smartdrive(backleftdrive, backrightdrive, inertialSensor, 320, 260, 280);

vex::motor flywheel = vex::motor(PORT10, true);
vex::motor expander = vex::motor(PORT9);
vex::motor intake = vex::motor(PORT6);
vex::motor roller = vex::motor(PORT15);

vex::controller rc = vex::controller();

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}
