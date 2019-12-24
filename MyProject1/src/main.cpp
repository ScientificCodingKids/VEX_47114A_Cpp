/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Mon Mar 25 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
// Drive for Distance
// This program will instruct your robot to move 20 cm in the forward direction at half power.
// There is a two second pause at the beginning of the program.
//
// Robot Configuration: 
// [Smart Port]    [Name]        [Type]           [Description]       [Reversed]
// Motor Port 1    LeftMotor     V5 Smart Motor    Left side motor     false
// Motor Port 10   RightMotor    V5 Smart Motor    Right side motor    true
//
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// define your global instances of motors and other devices here
vex::motor LeftMotor   = vex::motor( vex::PORT1 );
vex::motor RightMotor  = vex::motor( vex::PORT10, true );
vex::motor LiftLeftLowMotor = vex::motor(vex::PORT2);
vex::motor LiftRightLowMotor = vex::motor(vex::PORT2);

vex::motor_group group_chloe = vex::motor_group(LeftMotor, RightMotor, LiftLeftLowMotor, LiftRightLowMotor);

int main() {
    // Wait 2 seconds or 2000 milliseconds before starting the program.
    vex::task::sleep( 2000 );
    // Print to the screen that the program has started.
    Brain.Screen.print("User Program has Started.");
    
    
    // Define variables. You may need to change these variables for this program to work for your robot.

    // wheelDiameter is the measurement of a wheel from edge to edge in centimeters.
    double wheelDiameterCM  = 10.16; 

    // travelTarget will define how far we want the robot to move in centimeters.
    double travelTargetCM = 20;      
    
    // Calculate wheel circumference: circumference = (wheelDiameter)(PI)
    double circumference = wheelDiameterCM * 3.141592;    

    // Now we know the robot will travel the Circumference per 360 degrees of rotation or (circumference / 360 degrees).
    // We also know that are target distance is defined as travelTargetCM, but we do not know the degrees to of rotation. (traveTargetCM / ?)
    // Using propotional reasoning we know (circumference / 360) = (travelTarget / degreesToRotate).
    // Solving for our unkown (degreesToRotate) we get: degreesToRotate = (360 / travelTargetCM) / circumference
    double degreesToRotate = (360 * travelTargetCM) / circumference;

    // All calculations are complete. Start the rest of the program.
    
    // Set the velocity of the left and right motors to 50% power.
    // This command will not make the motor spin.
    LeftMotor.setVelocity( 50, vex::velocityUnits::pct ); 
    RightMotor.setVelocity( 50, vex::velocityUnits::pct );
    
    // Rotate the left and right motor for degreesToRotate. 
    // This command must be non blocking.
    LeftMotor.rotateFor( degreesToRotate, vex::rotationUnits::deg, false ); 
    // This command is blocking so the program will wait here until the right motor is done.  
    RightMotor.rotateFor( degreesToRotate, vex::rotationUnits::deg ); 
    
    group_chloe.rotateFor(vex::directionType::fwd, 30, vex::rotationUnits::deg);

    LeftMotor.startRotateFor(vex::fwd, 30, vex::rotationUnits::deg);
    //The motors will brake once they reach their destination.
    
    // Print to the brain's screen that the program has ended.
    Brain.Screen.newLine();//Move the cursor to a new line on the screen.
    Brain.Screen.print( "User Program has Ended." );

    // Prevent main from exiting with an infinite loop.                        
    while(1) {
      // Sleep the task for a short amount of time to prevent wasted resources.
      vex::task::sleep(100);
    }
}
