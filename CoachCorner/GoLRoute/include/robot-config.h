using namespace vex;

extern brain Brain;

extern vex::motor backleftdrive;
extern vex::motor backrightdrive;
extern vex::motor frontleftdrive;
extern vex::motor frontrightdrive;

// extern vex::drivetrain dt;

extern vex::motor rightlift;
extern vex::motor leftlift;

extern vex::motor_group lift;

extern vex::motor leftintake;
extern vex::motor rightintake;

//extern vex::motor_group intake;
extern vex::controller rc;

extern inertial inertialSensor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
