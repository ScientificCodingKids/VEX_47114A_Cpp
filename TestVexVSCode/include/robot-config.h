using namespace vex;

extern brain Brain;
extern vex::motor intake;
extern vex::motor conveyor;

extern vex::digital_out mogomech;

extern vex::motor backleftdrive;
extern vex::motor backrightdrive;
extern vex::motor frontleftdrive;
extern vex::motor frontrightdrive;


extern vex::inertial inertialSensor;

// extern vex::motor cat;  // catapult

extern vex::controller rc;

//extern vex::vision visionSensor;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);
