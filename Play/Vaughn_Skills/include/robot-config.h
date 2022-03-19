using namespace vex;

extern brain Brain;

extern vex::motor backleftdrive;
extern vex::motor backrightdrive;
extern vex::motor frontleftdrive;
extern vex::motor frontrightdrive;

extern vex::motor_group leftdrive;
extern vex::motor_group rightdrive;

extern vex::inertial inertialSensor;

extern vex::drivetrain dt;
extern vex::smartdrive sdrive;

extern vex::motor rightlift;
extern vex::motor leftlift;

extern vex::motor_group lift;

extern vex::motor leftintake;
extern vex::motor rightintake;

//extern vex::motor_group intake;
extern vex::controller rc;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
