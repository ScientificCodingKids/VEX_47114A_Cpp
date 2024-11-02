/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       north                                                     */
/*    Created:      9/21/2024, 9:08:47 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

competition Competition;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;


// define your global instances of motors and other devices here
vex::motor intake = vex::motor(PORT5, true);

vex::motor backleftdrive = vex::motor(PORT3, true);
vex::motor backrightdrive = vex::motor(PORT4);
vex::motor frontleftdrive = vex::motor(PORT1, true);
vex::motor frontrightdrive = vex::motor(PORT2);

motor_group leftDrive = motor_group(backleftdrive, frontleftdrive);
motor_group rightDrive = motor_group(backrightdrive, frontrightdrive);

void pre_auton(void) {

}

void autonomous(void) {
    intake.spin(vex::directionType::fwd);
    leftDrive.setVelocity(50, velocityUnits::pct);
    rightDrive.setVelocity(50, velocityUnits::pct);

    leftDrive.spin(directionType::fwd);
    rightDrive.spin(directionType::fwd);
}

void usercontrol(void) {

}

int main() {

    Brain.Screen.printAt( 10, 50, "Hello V5" );
    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    pre_auton();
   
    while(1) {
        
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
