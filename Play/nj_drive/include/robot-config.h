using namespace vex;

extern brain Brain;

extern vex::motor backleftdrive;
extern vex::motor backrightdrive;
extern vex::motor frontleftdrive;
extern vex::motor frontrightdrive;

extern vex::motor leftlift;
extern vex::motor rightlift;

extern vex::motor intake;

extern vex::motor mogo;

extern vex::motor_group lift;

extern vex::motor_group dt;

extern vex::controller rc;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
