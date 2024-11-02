using namespace vex;

extern brain Brain;

extern competition Competition;

extern vex::motor intake;

extern vex::controller rc;

extern vex::motor backleftdrive;
extern vex::motor backrightdrive;
extern vex::motor frontleftdrive;
extern vex::motor frontrightdrive;

extern motor_group leftDrive;
extern motor_group rightDrive;

extern digital_out mogomech;
extern vex::inertial inertialSensor;
extern vex::motor conveyor;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void);


class Coord {
  public:
    double x, y;

    Coord(double x0, double y0): x(x0), y(y0) {;}
};  //class Coord
