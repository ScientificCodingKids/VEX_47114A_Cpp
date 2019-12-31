using namespace vex;

extern brain Brain;
extern motor leftDriveMotor, rightDriveMotor;
extern motor rightuplift, rightdownlift, leftuplift, leftdownlift;

extern drivetrain dt;
extern motor_group lift;

extern motor claw;
extern controller rc;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
