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

    void resetRot() {
        backleftdrive.resetRotation();
        backrightdrive.resetRotation();
        frontleftdrive.resetRotation();
        frontrightdrive.resetRotation();
    }

    double frxdriveVelocity (double speed, double tgtHeading, double wheelAngle = 45) {
        return speed * sin(degree2arc(90-wheelAngle-tgtHeading));
    }

    double flxdriveVelocity (double speed, double tgtHeading, double wheelAngle = 45) {
        return speed * sin(degree2arc(90-wheelAngle+tgtHeading));
    }

    double brxdriveVelocity (double speed, double tgtHeading, double wheelAngle = 45) {
        return speed * sin(degree2arc(90-wheelAngle+tgtHeading));
    }

    double blxdriveVelocity (double speed, double tgtHeading, double wheelAngle = 45) {
        return speed * sin(degree2arc(90-wheelAngle-tgtHeading));
    }

    Coord goStraight(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, double kp = 0.02, vex::brakeType bt = brake, Coord srcLoc = Coord(0.0, 0.0));

};  // class XDriveRobot

Coord XDriveRobot::goStraight(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, double kp, vex::brakeType bt, Coord srcLoc) {
  Coord currLoc = srcLoc;

  resetRot();

  // orientation of the wheels, default = 45
  double wheelAngle = 45;
  double wheelArc = degree2arc(45);

  // target orientation that the robot should cont. to travel at
  // tgtHeading is a parameter
  double targetArc = degree2arc(tgtHeading);
  
  // tracking distance
  double distToGo = dist; // distance more to travel
  double distTravelled = dist - distToGo; // distance already traveled
  double prevRot = backleftdrive.rotation(vex::rotationUnits::deg); // amount of rotation at the last run through
  double changedRot = backleftdrive.rotation(vex::rotationUnits::deg) - prevRot; // rotations passed since last loop

  // speed
  double finalSpeed = 10; // capping speed
  double speed = originalSpeed; // max speed
  double const adaptiveInterval = 10; // slow down/speed up interval

  
  // location tracking on the coordinate plane
  double dx = 0; // change in x coordinate since last loop
  double dy = 0; // change in y coordinate since last loop

  double xspeed = originalSpeed * cos(targetArc);
  double yspeed = originalSpeed * sin(targetArc);

  double blspeed = xSpeed - ySpeed;
  double brspeed = -xSpeed - ySpeed;
  double flspeed = -xSpeed + ySpeed;
  double frspeed = xSpeed + ySpeed;

  while (distToGo > 0) {

    double frsign = 1;
    double flsign = -1;
    double blsign = -1;
    double brsign = 1;

    // update other variables

    double headingError = inertialSensor.heading() - tgtHeading; // updates the error in heading

    // adjusts the heading error so that it is between -180 and 180
    if (headingError < -180) headingError = headingError + 360;
    if (headingError > 180) headingError = headingError - 360;

    // caps headingError so that adjustments are not too dramatic
    if (headingError < -15) headingError = -15;
    if (headingError > 15) headingError = 15;

    // by default, speed with no adjustments is as such
    speed = originalSpeed;

    if (distTravelled < adaptiveInterval) speed = originalSpeed * distTravelled / adaptiveInterval;
    if (distToGo < adaptiveInterval) speed = originalSpeed * (1 - (adaptiveInterval - distToGo) / adaptiveInterval);

    if (speed < finalSpeed) speed = finalSpeed;

    

    if (dt == vex::directionType::fwd) {
      leftdrive.setVelocity(speed * (1 - kp * headingError), vex::percentUnits::pct);
      rightdrive.setVelocity(speed * (1 + kp * headingError), vex::percentUnits::pct);
    } else {
      leftdrive.setVelocity(speed * (1 + kp * headingError), vex::percentUnits::pct);
      rightdrive.setVelocity(speed * (1 - kp * headingError), vex::percentUnits::pct);
    }
    leftdrive.spin(dt);
    rightdrive.spin(dt);

    vex::task::sleep(10);

    distToGo = dist - fabs(leftdrive.rotation(vex::rotationUnits::deg) / 360 * (4.0 * 3.1415269265));

    changedRotations = (leftdrive.rotation(vex::rotationUnits::deg) * 4.0 * 3.1415269265) / 360 - prevRot; // need scale, deg => inch

    dx = changedRotations * cos(90-inertialSensor.heading());
    dy = changedRotations * sin(90-inertialSensor.heading());

    prevRot = (leftdrive.rotation(vex::rotationUnits::deg) * 4.0 * 3.1415269265) / 360;

    currLoc.x = dx + currLoc.x;
    currLoc.y = dy + currLoc.y;
    
    distTravelled = dist - distToGo;
  }
  cout << "finalSpeed = " << speed << endl;
  Brain.Screen.print("finalSpeed = %f ", speed);
  leftdrive.stop(bt);
  rightdrive.stop(bt);
  cout << "ending coordinate = " << currLoc.x << ", " << currLoc.y << endl;
  Brain.Screen.print("ending coordinate = %f, %f", currLoc.x, currLoc.y);
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
