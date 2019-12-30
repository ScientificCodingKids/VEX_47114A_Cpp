// To complete the VEXcode V5 Text project upgrade process, please follow the
// steps below.
// 
// 1. You can use the Robot Configuration window to recreate your V5 devices
//   - including any motors, sensors, 3-wire devices, and controllers.
// 
// 2. All previous code located in main.cpp has now been commented out. You
//   will need to migrate this code to the new "int main" structure created
//   below and keep in mind any new device names you may have set from the
//   Robot Configuration window. 
// 
// If you would like to go back to your original project, a complete backup
// of your original (pre-upgraded) project was created in a backup folder
// inside of this project's folder.

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}

// // ---- START VEXCODE CONFIGURED DEVICES ----
// // Robot Configuration:
// // [Name]               [Type]        [Port(s)]
// // ---- END VEXCODE CONFIGURED DEVICES ----
// /*----------------------------------------------------------------------------*/
// /*                                                                            */
// /*    Module:       main.cpp                                                  */
// /*    Author:       {author}                                                  */
// /*    Created:      {date}                                                    */
// /*    Description:  V5 project                                                */
// /*                                                                            */
// /*----------------------------------------------------------------------------*/
// #include "vex.h"
// 
// using namespace vex;
// 
// // A global instance of vex::brain used for printing to the V5 brain screen
// vex::brain       Brain;
// // A global instance of vex::competition
// vex::competition Competition;
// 
// // define your global instances of motors and other devices here
// vex::motor leftDriveMotor = vex::motor(vex::PORT10);
// vex::motor rightDriveMotor = vex::motor(vex::PORT1, true);
// 
// vex::drivetrain dt = vex::drivetrain(leftDriveMotor, rightDriveMotor);
// 
// vex::motor rightuplift = vex::motor(vex::PORT4);
// vex::motor rightdownlift = vex::motor(vex::PORT7, true);
// vex::motor leftuplift = vex::motor(vex::PORT6, true);
// vex::motor leftdownlift = vex::motor(vex::PORT5);
// 
// vex::motor_group lift = vex::motor_group(rightuplift, rightdownlift, leftdownlift, leftuplift);
// 
// /*---------------------------------------------------------------------------*/
// /*                          Pre-Autonomous Functions                         */
// /*                                                                           */
// /*  You may want to perform some actions before the competition starts.      */
// /*  Do them in the following function.  You must return from this function   */
// /*  or the autonomous and usercontrol tasks will not be started.  This       */
// /*  function is only called once after the cortex has been powered on and    */ 
// /*  not every time that the robot is disabled.                               */
// /*---------------------------------------------------------------------------*/
// 
// void pre_auton( void ) {
//   // All activities that occur before the competition starts
//   // Example: clearing encoders, setting servo positions, ...
//   
// }
// 
// 
// void autonomous( void ) {
//   // ..........................................................................
//   // Insert autonomous user code here.
//   // ..........................................................................
// 
// }
// 
// /*---------------------------------------------------------------------------*/
// /*                                                                           */
// /*                              User Control Task                            */
// /*                                                                           */
// /*  This task is used to control your robot during the user control phase of */
// /*  a VEX Competition.                                                       */
// /*                                                                           */
// /*  You must modify the code to add your own robot specific commands here.   */
// /*---------------------------------------------------------------------------*/
// 
// void usercontrol( void ) {
//   // User control code here, inside the loop
//   while (1) {
//     // This is the main execution loop for the user control program.
//     // Each time through the loop your program should update motor + servo 
//     // values based on feedback from the joysticks.
// 
//     // ........................................................................
//     // Insert user code here. This is where you use the joystick values to 
//     // update your motors, etc.
//     // ........................................................................
//  
//     vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
//   }
// }
// 
// //
// // Main will set up the competition functions and callbacks.
// //
// int main() {
//     //Set up callbacks for autonomous and driver control periods.
//     Competition.autonomous( autonomous );
//     Competition.drivercontrol( usercontrol );
//     
//     //Run the pre-autonomous function. 
//     pre_auton();
//        
//     //Prevent main from exiting with an infinite loop.                        
//     while(1) {
//       vex::task::sleep(100);//Sleep the task for a short amount of time to prevent wasted resources.
//     }    
// }