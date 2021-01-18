#include "vex.h"

using namespace vex;

// A global instance of brain used for printing to the V5 brain screen
brain Brain;

vex::motor leftDriveMotor = vex::motor(vex::PORT10, true);
vex::motor rightDriveMotor = vex::motor(vex::PORT1);

vex::drivetrain dt = vex::drivetrain(leftDriveMotor, rightDriveMotor);

vex::motor rightuplift = vex::motor(vex::PORT5);  // 4->5
vex::motor leftuplift = vex::motor(vex::PORT6, true);

vex::motor_group lift = vex::motor_group(leftuplift, rightuplift);


vex::motor upclaw = vex::motor(vex::PORT18);
vex::motor downclaw = vex::motor(vex::PORT20, true);

vex::motor_group claw = vex::motor_group(downclaw);
vex::controller rc = vex::controller();

vex::inertial inertialSensor = vex::inertial(vex::PORT16);

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}
