#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

vex::motor backleftdrive = vex::motor(PORT1);
vex::motor backrightdrive = vex::motor(PORT19, true);
vex::motor frontleftdrive = vex::motor(PORT2);
vex::motor frontrightdrive = vex::motor(PORT20, true);

vex::motor leftlift = vex::motor(PORT5);
vex::motor rightlift = vex::motor(PORT6, true);

vex::motor intake = vex::motor(PORT8);

vex::motor mogo = vex::motor(PORT15);

vex::motor_group lift = vex::motor_group(leftlift, rightlift);

vex::motor_group dt = vex::motor_group(backleftdrive, backrightdrive, frontleftdrive, frontrightdrive);

vex::controller rc = vex::controller();


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}