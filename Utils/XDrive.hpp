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
    XDriveRobot(vex::motor& bl, vex::motor& br, vex::motor& fr, vex::motor& fl, vex::brain& brn, vex::inertial& ins, vex::motor& fly, vex::motor& ind, vex::motor& intk): 
        backleftdrive(bl), backrightdrive(br), frontrightdrive(fr), frontleftdrive(fl), Brain (brn), inertialSensor(ins), flywheel(fly), indexer(ind), intake(intk) {;} 
    vex::motor& backleftdrive;
    vex::motor& backrightdrive;
    vex::motor& frontrightdrive;
    vex::motor& frontleftdrive;

    vex::brain& Brain;
    vex::inertial& inertialSensor;

    vex::motor& flywheel;
    vex::motor& indexer;
    vex::motor& intake;

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

    void stop(vex::brakeType bt) {
        backleftdrive.stop(bt);
        backrightdrive.stop(bt);
        frontleftdrive.stop(bt);
        frontrightdrive.stop(bt);
    }

};  // class XDriveRobot

double sign (double x) {
    if (x<0) return -1;
    else return 1;
}

double logDrive (double cv, double c = 1.5) {
    // map [-100, 100] to [-100, 100]
  return pow(fabs(cv/100.), c) * sign(cv) * 100.0;
}

double logDriveT(double cv) { // less intense, for turning
    return cv;
  //return pow(fabs(cv), 1.5) / (sign(cv)*sqrt(50));
}

void autonWithXD(XDriveRobot& robot, double tgtHeading, double speed, double spinSpeed, RollingScreen& rs, double kp = 0.01) {
    for (int t=0; t<50; ++t) {
        double h = robot.inertialSensor.heading();

        double xSpeed = speed * cos(degree2arc(tgtHeading));
        double ySpeed = speed * sin(degree2arc(tgtHeading));

        spinSpeed = spinSpeed - kp * (h - tgtHeading);

        
        rs.print("user speed: %.1f, %.1f, %.1f | %.1f, %.1f", xSpeed, ySpeed, spinSpeed, h, tgtHeading);

        double blspeed = xSpeed - ySpeed + spinSpeed;
        double brspeed = -xSpeed - ySpeed - spinSpeed;
        double flspeed = -xSpeed + ySpeed + spinSpeed;
        double frspeed = xSpeed + ySpeed - spinSpeed;

        robot.move(blspeed, brspeed, flspeed, frspeed, rs);

        task::sleep(10);
    }  // for t

    robot.stop(vex::brakeType::hold);

}  //autonWithXD

void driveWithXD(XDriveRobot& robot, vex::controller& rc, RollingScreen& rs, double kp) {

    while (1) {

        double h = robot.inertialSensor.heading();

        double xSpeed = logDrive(rc.Axis4.position(vex::percentUnits::pct)); // run experiment to see which side is positive/negative
        double ySpeed = logDrive(rc.Axis3.position(vex::percentUnits::pct));
        double spinSpeed = -logDriveT(rc.Axis1.position(vex::percentUnits::pct)*3/4);
        
        rs.print("user speed: %.1f, %.1f, %.f", xSpeed, ySpeed, spinSpeed);

        double blspeed = xSpeed - ySpeed + spinSpeed;
        double brspeed = -xSpeed - ySpeed - spinSpeed;
        double flspeed = -xSpeed + ySpeed + spinSpeed;
        double frspeed = xSpeed + ySpeed - spinSpeed;

        robot.move(blspeed, brspeed, flspeed, frspeed, rs);

        if (rc.ButtonY.pressing()) {
            robot.flywheel.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
        }
        else if (rc.ButtonX.pressing()) {
            robot.flywheel.stop(vex::brakeType::coast);
        }

        if (rc.ButtonUp.pressing()) {
            robot.intake.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
        }
        else if (rc.ButtonDown.pressing()) {
            robot.intake.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
        }
        else if (rc.ButtonLeft.pressing()) {
            robot.intake.stop(vex::brakeType::coast);
        }

        vex::task::sleep(10);
    }
}

#endif // _Xdrive_hpp_
