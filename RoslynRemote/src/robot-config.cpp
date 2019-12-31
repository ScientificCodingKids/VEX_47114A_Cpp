#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

vex::motor leftDriveMotor = vex::motor(vex::PORT10);
vex::motor rightDriveMotor = vex::motor(vex::PORT1, true);

vex::drivetrain dt = vex::drivetrain(leftDriveMotor, rightDriveMotor);

vex::motor rightuplift = vex::motor(vex::PORT4);
vex::motor rightdownlift = vex::motor(vex::PORT7, true);
vex::motor leftuplift = vex::motor(vex::PORT6, true);
vex::motor leftdownlift = vex::motor(vex::PORT5);

vex::motor_group lift = vex::motor_group(rightuplift, rightdownlift, leftdownlift, leftuplift);

vex::motor claw = vex::motor(vex::PORT20);

vex::controller rc = vex::controller();

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}