#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

vex::motor backleftdrive = vex::motor(PORT10, true);
vex::motor backrightdrive = vex::motor(PORT1);
vex::motor frontleftdrive = vex::motor(PORT16, true);
vex::motor frontrightdrive = vex::motor(PORT15);

vex::motor_group leftdrive = vex::motor_group(backleftdrive, frontleftdrive);
vex::motor_group rightdrive = vex::motor_group(backrightdrive, frontrightdrive);

vex::drivetrain dt = vex::drivetrain(leftdrive, rightdrive, 320, 260, 280);

vex::motor rightlift = vex::motor(PORT4, true);
vex::motor leftlift = vex::motor(PORT9);

vex::motor_group lift = vex::motor_group(rightlift, leftlift);

vex::motor rightintake = vex::motor(PORT20, true);
vex::motor leftintake = vex::motor(PORT18);

vex::inertial inertialSensor = vex::inertial(vex::PORT11);

vex::controller rc = vex::controller();

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}
