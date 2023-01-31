#ifndef _XDrive_hpp_
#define _XDrive_hpp_

// #include "vex.h"  -- SHOULD NOT DEPEND ON A SPECIFIC PROJECT!!!

#include "C:/Program Files (x86)/VEX Robotics/VEXcode Pro V5/sdk/vexv5/include/vex_motor.h"
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
    XDriveRobot(vex::motor& bl, vex::motor& br, vex::motor& fl, vex::motor& fr, vex::brain& brn, vex::inertial& ins, vex::motor& fly, vex::motor& exp, vex::motor& intk, vex::motor& roll): 
        backleftdrive(bl), backrightdrive(br), frontleftdrive(fr), frontrightdrive(fl), Brain (brn), inertialSensor(ins), flywheel(fly), expander(exp), intake(intk), roller(roll) {
            ;
    }
    vex::motor& backleftdrive;
    vex::motor& backrightdrive;
    vex::motor& frontleftdrive;
    vex::motor& frontrightdrive;

    vex::brain& Brain;
    vex::inertial& inertialSensor;

    vex::motor& flywheel;
    vex::motor& expander;
    vex::motor& intake;
    vex::motor& roller;

    void calibrate () {
        inertialSensor.calibrate();
        vex::task::sleep(1500);
    }

    void move(double blspeed, double brspeed, double flspeed, double frspeed) {

        backleftdrive.spin(vex::directionType::fwd, blspeed, vex::velocityUnits::pct);
        backrightdrive.spin(vex::directionType::fwd, brspeed, vex::velocityUnits::pct);
        frontrightdrive.spin(vex::directionType::fwd, frspeed, vex::velocityUnits::pct);
        frontleftdrive.spin(vex::directionType::fwd, flspeed, vex::velocityUnits::pct);


    }  // move()

    void stop(vex::brakeType bt) {
        backleftdrive.stop(bt);
        backrightdrive.stop(bt);
        frontleftdrive.stop(bt);
        frontrightdrive.stop(bt);
    }

    void resetRot() {
        backleftdrive.resetRotation();
        backrightdrive.resetRotation();
        frontleftdrive.resetRotation();
        frontrightdrive.resetRotation();
    }

    Coord goStraight(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, double kp = 0.02, vex::brakeType bt = brake, Coord srcLoc = Coord(0.0, 0.0));

};  // class XDriveRobot

double sin_by_deg(double x) {
    return sin(x/M_PI);
}

double cos_by_deg(double x) {
    return cos(x/M_PI);
}

Coord XDriveRobot::goStraight(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, double kp, vex::brakeType bt, Coord srcLoc) {
    RollingScreen rs(this->Brain.Screen);

    Coord currLoc = srcLoc;
    
    Coord destLoc(srcLoc.x + dist * cos(90 - tgtHeading), srcLoc.y + dist * sin(90 - tgtHeading));

    double refDiameter = 4.0; // inch; for the wheel attached to the refMotor

    resetRot();

    // orientation of the wheels, default = 45
    double wheelAngle = 45;

    // tracking distance
    double distToGo = dist; // distance more to travel

    // speed
    double finalSpeed = 10; // lowest speed to avoid stall
    double const adaptiveInterval = 10; // slow down/speed up interval

    double frsign = 1;
    double flsign = -1;
    double blsign = -1;
    double brsign = 1;

    double msecs = 10.0; // looping interval

    while (distToGo > 0) {
        // update other variables
        double speed = originalSpeed; // compute current speed, which is subject to ramp up/down

        double headingError = inertialSensor.heading() - tgtHeading; // updates the error in heading

        // adjusts the heading error so that it is between -180 and 180
        if (headingError < -180) headingError = headingError + 360;
        if (headingError > 180) headingError = headingError - 360;

        // caps headingError so that adjustments are not too dramatic
        if (headingError < -15) headingError = -15;
        if (headingError > 15) headingError = 15;

        // by default, speed with no adjustments is as such
        speed = originalSpeed;

        if (distToGo > dist - adaptiveInterval) speed = originalSpeed * (dist - distToGo) / adaptiveInterval;
        if (distToGo < adaptiveInterval) speed = originalSpeed * (1 - (adaptiveInterval - distToGo) / adaptiveInterval);

        if (speed < finalSpeed) speed = finalSpeed;

        // effective speed is computed at this point -- NO MORE adj
        double xSpeed = speed * cos_by_deg(90 - tgtHeading);
        double ySpeed = speed * sin_by_deg(90 - tgtHeading);


        // based on motor config: +, -, -, +
        double blspeed = xSpeed - ySpeed;
        double brspeed = -xSpeed - ySpeed;
        double flspeed = xSpeed + ySpeed;
        double frspeed = -xSpeed + ySpeed;

        double kpHeadingErr = kp * headingError;

        move(blspeed * (1 + kpHeadingErr * blsign), 
            brspeed * (1 + kpHeadingErr * brsign), 
            flspeed * (1 + kpHeadingErr * flsign), 
            frspeed * (1 + kpHeadingErr * frsign)
            );

        // cannot directly use "speed" as they are relative

        double blDist = backleftdrive.rotation(vex::rotationUnits::deg) / 360. * M_PI * refDiameter;
        double brDist = backrightdrive.rotation(vex::rotationUnits::deg) / 360. * M_PI * refDiameter;
        double flDist = frontleftdrive.rotation(vex::rotationUnits::deg) / 360. * M_PI * refDiameter;
        double frDist = frontrightdrive.rotation(vex::rotationUnits::deg) / 360. * M_PI * refDiameter;

        double dx = (blDist - brDist + flDist - frDist) / 4.0;
        double dy = (-blDist - brDist + flDist + frDist) / 4.0;

        currLoc.x = dx + currLoc.x;
        currLoc.y = dy + currLoc.y;

        distToGo = sqrt( (destLoc.x - currLoc.x)*(destLoc.x - currLoc.x) + (destLoc.y - currLoc.y) * (destLoc.y - currLoc.y) );

        vex::task::sleep(msecs);

    }  // while

    stop(bt);

    //cout << "ending coordinate = " << currLoc.x << ", " << currLoc.y << endl;
    //Brain.Screen.print("ending coordinate = %f, %f", currLoc.x, currLoc.y);
    return currLoc;
} 

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

void autonWithXD(XDriveRobot& robot) {

    robot.move(-50, -50, 50, 50);
    vex::task::sleep(1000);
    robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    robot.roller.stop(vex::brakeType::coast);
    // robot.move(50, 50, -50, -50);
    // vex::task::sleep(2300);
    // robot.stop(vex::brakeType::coast);
    // robot.move(50, -50, -50, 50);
    // robot.stop(vex::brakeType::coast);
    // robot.expander.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
    // vex::task::sleep(1000);
    // robot.expander.stop(vex::brakeType::coast);
    

}  //autonWithXD

void driveWithXD(XDriveRobot& robot, vex::controller& rc, double kp) {
    RollingScreen rs(robot.Brain.Screen);

    while (1) {
        double xSpeed = logDrive(rc.Axis4.position(vex::percentUnits::pct)); // run experiment to see which side is positive/negative
        double ySpeed = logDrive(rc.Axis3.position(vex::percentUnits::pct));

        // force move along x or y direction, no striffing
        if (fabs(xSpeed) > 2.0 * fabs(ySpeed)) {
            ySpeed = 0;
        }
        else {
            if (fabs(xSpeed) < 0.5 * fabs(ySpeed)) {
                xSpeed = 0;
            }
            else {
                xSpeed = 0;
                ySpeed = 0;
            }
        }

        double spinSpeed = logDriveT(rc.Axis1.position(vex::percentUnits::pct)*3/4);
        
        rs.print("user speed: %.1f, %.1f, %.f", xSpeed, ySpeed, spinSpeed);

        // based on motor config: +, -, -, +
        double blspeed = xSpeed - ySpeed - spinSpeed;
        double brspeed = -xSpeed - ySpeed + spinSpeed;
        double flspeed = xSpeed + ySpeed + spinSpeed;
        double frspeed = -xSpeed + ySpeed - spinSpeed;

        robot.move(blspeed, brspeed, flspeed, frspeed);

        if (rc.ButtonY.pressing()) {
            robot.flywheel.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
        }
        else if (rc.ButtonLeft.pressing()){
            robot.flywheel.spin(vex::directionType::fwd, 40, vex::velocityUnits::pct);
        }
        else {
            robot.flywheel.stop(vex::brakeType::coast);
        }

        if (rc.ButtonL1.pressing()) {
            robot.intake.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
        }
        else if (rc.ButtonL2.pressing()) {
            robot.intake.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
        }
        else {
            robot.intake.stop(vex::brakeType::coast);
        }

        if (rc.ButtonR1.pressing()) {
            robot.roller.spin(vex::directionType::fwd, 70, vex::velocityUnits::pct);
        }
        else if (rc.ButtonR2.pressing()) {
            robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
        }
        else {
            robot.roller.stop(vex::brakeType::coast);
        }

        if (rc.ButtonUp.pressing()) {
            robot.expander.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
        }
        else if (rc.ButtonDown.pressing()) {
            robot.expander.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
        }
        else {
            robot.expander.stop(vex::brakeType::coast);
        }

        vex::task::sleep(10);
    }
}

#endif // _Xdrive_hpp_
