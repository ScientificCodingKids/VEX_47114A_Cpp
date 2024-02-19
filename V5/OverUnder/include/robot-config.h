using namespace vex;

extern brain theBrain;

extern vex::motor backleftdrive;
extern vex::motor backrightdrive;
extern vex::motor frontleftdrive;
extern vex::motor frontrightdrive;

extern vex::motor_group leftdrive;
extern vex::motor_group rightdrive;

extern vex::inertial inertialSensor;

extern vex::motor cat;  // catapult

extern vex::motor intakeMotor;
extern vex::motor intakeLifter;

extern vex::controller rc;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
