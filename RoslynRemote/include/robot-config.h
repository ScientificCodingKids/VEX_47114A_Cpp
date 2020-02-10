using namespace vex;

extern brain Brain;
extern motor leftDriveMotor, rightDriveMotor;
extern motor rightuplift, rightdownlift, leftuplift, leftdownlift;
extern motor upclaw;
extern motor downclaw;

extern drivetrain dt;
extern motor_group lift;

extern motor_group claw;
extern controller rc;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
