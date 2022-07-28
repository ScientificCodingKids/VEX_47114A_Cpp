#ifndef _XDrive_hpp_
#define _XDrive_hpp_

// #include "vex.h"  -- SHOULD NOT DEPEND ON A SPECIFIC PROJECT!!!

#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>
#include <cassert>

#include "nutils.hpp"
using namespace vex;
using namespace std;


#ifndef M_PI
#define M_PI = 3.14159265
#endif

class XDriveRobot {
    public:
    XDriveRobot(vex::motor& bl, vex::motor& br, vex::motor& fr, vex::motor& fl, vex::brain& brn, vex::inertial& ins): 
        backleftdrive(bl), backrightdrive(br), frontrightdrive(fr), frontleftdrive(fl), Brain (brn), inertialSensor(ins) {;} 
    vex::motor& backleftdrive;
    vex::motor& backrightdrive;
    vex::motor& frontrightdrive;
    vex::motor& frontleftdrive;

    vex::brain Brain;
    vex::inertial inertialSensor;

    void calibrate () {
        inertialSensor.calibrate();
        vex::task::sleep(1500);
    }

    void move(double blspeed, double brspeed, double flspeed, double frspeed, RollingScreen& rs) {

        backleftdrive.spin(vex::directionType::fwd, blspeed, vex::velocityUnits::pct);
        backrightdrive.spin(vex::directionType::fwd, brspeed, vex::velocityUnits::pct);
        frontrightdrive.spin(vex::directionType::fwd, frspeed, vex::velocityUnits::pct);
        frontleftdrive.spin(vex::directionType::fwd, flspeed, vex::velocityUnits::pct);

        rs.print("motor speed: %.1f, %.1f, %.f, %.1f", blspeed, brspeed, flspeed, frspeed);

    }  // move()

};  // class XDriveRobot

double sign (double x) {
    if (x<0) return -1;
    else return 1;
}

double logDrive (double cv) {
  return pow(fabs(cv), 1.5) / (sign(cv)*sqrt(100));
}

double logDriveT(double cv) { // less intense, for turning
    return cv;
  //return pow(fabs(cv), 1.5) / (sign(cv)*sqrt(50));
}

void driveWithXD(XDriveRobot& robot, vex::controller& rc, RollingScreen& rs, double kp) {
    while (1) {
        double h = robot.inertialSensor.heading();

        double xSpeed = logDrive(rc.Axis4.position(vex::percentUnits::pct)); // run experiment to see which side is positive/negative
        double ySpeed = logDrive(rc.Axis3.position(vex::percentUnits::pct));
        double spinSpeed = -logDriveT(rc.Axis1.position(vex::percentUnits::pct)*3/4);

        // double intendAngle = arc2degree(atan(abs(ySpeed/xSpeed)));
        // if (xSpeed >= 0 && ySpeed >= 0) intendAngle = 90 - intendAngle;
        // if (xSpeed >=0 && ySpeed < 0) intendAngle = 90 + intendAngle;
        // if (xSpeed < 0 && ySpeed >= 0) intendAngle = 270 + intendAngle;
        // if (xSpeed < 0 && ySpeed < 0) intendAngle = 270 -
        // double actualAngle = h;
        
        
        rs.print("user speed: %.1f, %.1f, %.f", xSpeed, ySpeed, spinSpeed);

        double blspeed = xSpeed - ySpeed + spinSpeed;
        double brspeed = -xSpeed - ySpeed - spinSpeed;
        double flspeed = -xSpeed + ySpeed + spinSpeed;
        double frspeed = xSpeed + ySpeed - spinSpeed;

        // double maxAxis = MAX(fabs(xSpeed), fabs(ySpeed), fabs(spinSpeed)); //Find the maximum input given by the controller's axes and the angle corrector
        // float maxOutput = MAX(fabs(blspeed), fabs(brspeed), fabs(flspeed), fabs(frspeed)); //Find the maximum output that the drive program has calculated

        // if (maxOutput == 0 || maxAxis == 0) {
        //     normalizer = 0;
        // } 
        // else {
        //     normalizer = maxAxis / maxOutput; 
        // }

        // blspeed *= normalizer;
        // brspeed *= normalizer;
        // flspeed *= normalizer;
        // frspeed *= normalizer;

        robot.move(blspeed, brspeed, flspeed, frspeed, rs);

        vex::task::sleep(10);
    }
}


// void xStop (xDriveRobot& robot) {
//     robot.backleftdrive.stop();
//     robot.backrightdrive.stop();
//     robot.frontleftdrive.stop();
//     robot.frontrightdrive.stop();
// }



// void xDriveUsercontrol (xDriveRobot& robot) {
//     calibrate(robot.inertialSensor);
//     double motorSpeed[4]; // bl-0, br-1, fl-2, fr-3


//     while (1) {
//         double h = robot.inertialSensor.heading();
//         double controllerX = logDrive(rc.Axis4.position(vex::percentUnits::percent)); // run experiment to see which side is positive/negative
//         double controllerY = logDrive(rc.Axis3.position(vex::percentUnits::percent));
//         double controllerZ = logDriveT(rc.Axis1.position(vex::percentUnits::percent)/2);

//         float magnitude = sqrt((joyX*joyX) + (joyY*joyY)) / M_SQRT2;  // modulus/sqrt(2)
//         float angle = atan2f(joyX, joyY) + (gyroAngle*(M_PI/180));

//         float x2 = magnitude * cos(angle); 
//         float y2 = magnitude * sin(angle);

//         motorSpeed[0] = x2 + y2 + controllerZ;
//         motorSpeed[1] = x2 + y2 + controllerZ;
//         motorSpeed[2] = x2 + y2 + controllerZ;
//         motorSpeed[3] = x2 + y2 + controllerZ;

//         float maxAxis = MAX(fabs(controllerX), fabs(controllerY), fabs(controllerZ)); 
//         float maxOutput = MAX(fabs(motorSpeed[0]), fabs(motorSpeed[1]), fabs(motorSpeed[2]), fabs(motorSpeed[3])); 

//         if (maxOutput == 0 || maxAxis == 0) {
//             normalizer = 0; //Prevent the undefined value for normalizer
//         } else {
//             normalizer = maxAxis / maxOutput; //calculate normalizer
//         }

//         for (int i = 0; i <= 3; i++) {
//             motorSpeed[i] *= normalizer; //caps motor speeds to the greatest input without losing the ratio between each speed, so as to not warp the direction of movement too much.
//         }

//         robot.backleftdrive.spin(forward, motorSpeed[0], vex::percentUnits::percent);
//         robot.backrightdrive.spin(forward, motorSpeed[1], vex::percentUnits::percent);
//         robot.frontleftdrive.spin(forward, motorSpeed[2], vex::percentUnits::percent);
//         robot.frontrightdrive.spin(forward, motorSpeed[3], vex::percentUnits::percent);
   
//         vex::task::sleep(10);
//     }
// }

#endif // _Xdrive_hpp_
