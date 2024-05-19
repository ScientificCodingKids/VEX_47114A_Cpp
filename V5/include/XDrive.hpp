#ifndef _XDrive_hpp_
#define _XDrive_hpp_

// #include "vex.h"  -- SHOULD NOT DEPEND ON A SPECIFIC PROJECT!!!

#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>
#include <cassert>
#include <algorithm>

#include "nutils.hpp"
using namespace vex;
using namespace std;


#ifndef M_PI
#define M_PI = 3.14159265
#endif

class XDriveRobot {
    public:
    XDriveRobot(vex::motor& bl, vex::motor& br, vex::motor& fl, vex::motor& fr, vex::brain& brn, vex::inertial& ins, vex::motor& fly, vex::motor& exp, vex::motor& intk, vex::motor& roll): 
        backleftdrive(bl), backrightdrive(br), frontleftdrive(fl), frontrightdrive(fr), Brain (brn), inertialSensor(ins), flywheel(fly), expander(exp), intake(intk), roller(roll) {
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
        vex::task::sleep(3500);
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

};  // class XDriveRobot

double sin_by_deg(double x) {
    return sin(x * M_PI / 180.);
}

double cos_by_deg(double x) {
    return cos(x * M_PI / 180.);
}

Coord XDriveRobot::goStraight(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, double kp, vex::brakeType bt, Coord srcLoc) {
    //RollingScreen rs(this->Brain.Screen);
    double startHeading = inertialSensor.heading();

    Coord currLoc = srcLoc;
    
    Coord destLoc(srcLoc.x + dist * cos_by_deg(90 - tgtHeading), srcLoc.y + dist * sin_by_deg(90 - tgtHeading));

    double refDiameter = 4.0; // inch; for the wheel attached to the refMotor

    resetRot();

    // orientation of the wheels, default = 45
    //double wheelAngle = 45;

    // tracking distance
    double distToGo = dist; // distance more to travel

    // speed
    double finalSpeed = 10; // lowest speed to avoid stall
    double const adaptiveInterval = 10; // slow down/speed up interval

    double blsign = -1;
    double brsign = 1;
    double flsign = 1;
    double frsign = -1;

    double msecs = 10.0; // looping interval

    double blDistPrev = 0.0;
    double brDistPrev = 0.0;
    double flDistPrev = 0.0;
    double frDistPrev = 0.0;

    while (distToGo > 0) {
        // update other variables
        double speed = originalSpeed; // compute current speed, which is subject to ramp up/down
        double currHeading = inertialSensor.heading();

        double headingError = currHeading - startHeading; // updates the error in heading

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
        double xSpeed = speed * cos_by_deg(90 - tgtHeading + startHeading);
        double ySpeed = speed * sin_by_deg(90 - tgtHeading + startHeading);


        // based on motor config: +, -, -, +
        double blspeed = xSpeed - ySpeed;
        double brspeed = -xSpeed - ySpeed;
        double flspeed = xSpeed + ySpeed;
        double frspeed = -xSpeed + ySpeed;

        double kpHeadingErr = -kp * headingError;

        double moveSign = (dt == directionType::fwd) ? 1.0 : -1.0;

        move(moveSign * (blspeed + speed * kpHeadingErr * blsign),
            moveSign * (brspeed + speed * kpHeadingErr * brsign),
            moveSign * (flspeed + speed * kpHeadingErr * flsign),
            moveSign * (frspeed + speed * kpHeadingErr * frsign)
        );


        // cannot directly use "speed" as they are relative

        double blDist = backleftdrive.rotation(vex::rotationUnits::deg) / 360. * M_PI * refDiameter;
        double brDist = backrightdrive.rotation(vex::rotationUnits::deg) / 360. * M_PI * refDiameter;
        double flDist = frontleftdrive.rotation(vex::rotationUnits::deg) / 360. * M_PI * refDiameter;
        double frDist = frontrightdrive.rotation(vex::rotationUnits::deg) / 360. * M_PI * refDiameter;

        double blDD = blDist - blDistPrev;
        double brDD = brDist - brDistPrev;
        double flDD = flDist - flDistPrev;
        double frDD = frDist - frDistPrev;

        double dx = (blDD - brDD + flDD - frDD) / 4.0;
        double dy = (-blDD - brDD + flDD + frDD) / 4.0;

        currLoc.x = dx + currLoc.x;
        currLoc.y = dy + currLoc.y;

        double dr = sqrt(dx*dx + dy*dy);
        distToGo -= dr;  // assume the actual path follows the straightline
        //rs.print("(%.1f, %.1f, %.1f, %.1f), ke=%.1f, dx=%.1f, %.1f", blspeed, brspeed, flspeed, frspeed, kpHeadingErr, dx, dy);
        
        //rs.print("%.1f, %.1f => %.1f, %.1f; %.1f, %.1f; %.1f", currLoc.x, currLoc.y, dr, distToGo, startHeading, currHeading, kpHeadingErr);
        
        vex::task::sleep(msecs);

        blDistPrev = blDist;
        brDistPrev = brDist;
        flDistPrev = flDist;
        frDistPrev = frDist;
    }  // while

    stop(bt);

    //cout << "ending coordinate = " << currLoc.x << ", " << currLoc.y << endl;
    //Brain.Screen.print("ending coordinate = %f, %f", currLoc.x, currLoc.y);
    return currLoc;
}  // goStraight()

Coord XDriveRobot::makeTurn(double tgtHeading, bool turnClockwise, double speed, double tol, brakeType bt, Coord srcLoc)
{
    // similar to regular drivetrain's makeTurn()
    // assume rotation is "in-place" for now, i.e. it does not change robot's location

    RollingScreen rs(Brain.Screen);

    Coord currLoc = srcLoc;

    resetRot();

    double degreeToGo = tgtHeading - inertialSensor.heading();

    if (degreeToGo < 0) {
        degreeToGo = degreeToGo + 360.0;
    }

    double CWDegreeToGo = degreeToGo;
    double CCWDegreeToGo = 360 - degreeToGo;

    double currentTurnClockwise = turnClockwise;

    while (degreeToGo > tol) {
        double ch = inertialSensor.heading();

        //1. compute cw, ccw degreeToGo
        CWDegreeToGo = tgtHeading - ch;
        CCWDegreeToGo = 360 - CWDegreeToGo;

        if (CWDegreeToGo < 0) {
            CWDegreeToGo = CWDegreeToGo + 360;
            CCWDegreeToGo = 360 - CWDegreeToGo;
        }

        assert(CWDegreeToGo >= 0 && CWDegreeToGo < 360);
        assert(CCWDegreeToGo >= 0 && CCWDegreeToGo < 360);

        //2. determine rotation direction and degreeToGo
        
        if (CWDegreeToGo < 30 || CCWDegreeToGo < 30) {
            currentTurnClockwise = CWDegreeToGo < CCWDegreeToGo;
        }
        else {
            currentTurnClockwise = turnClockwise;
        }
            
        degreeToGo = (currentTurnClockwise)? CWDegreeToGo: CCWDegreeToGo;

        // if kp is larger, correction is greater; if kp is smaller, correction is smaller

        double spinSpeed = speed;

        if (degreeToGo < 45) {
            spinSpeed = 10; // speed*(1.0-(30.0 - degreeToGo)/30.0);
        }

        spinSpeed = std::max(2.0, spinSpeed);

        //rs.print("h=%.1f; deg2Go=%.1f, sp=%.1f", ch, degreeToGo, spinSpeed);
        // 3. set motor speed and direction
        //Brain.Screen.print("%f, %d \n", currentSpeed, isClose);
        //cout << ch << ": [ " << CWDegreeToGo << ", " << CCWDegreeToGo << "]" << degreeToGo << ", " << headingError << ", " << currentSpeed << "; " << isClose << "; " << currentTurnClockwise << endl;  // print to terminal
        // based on motor config: +, -, -, +
        double blspeed = -spinSpeed;
        double brspeed = spinSpeed;
        double flspeed = spinSpeed;
        double frspeed = -spinSpeed;

        if (turnClockwise) {
            move(blspeed, brspeed, flspeed, frspeed);
        }
        else {
            move(-blspeed, -brspeed, -flspeed, -frspeed);
        }

        //update coordinates: skipped

        vex::task::sleep(15);
    }
    
    stop(bt);
    //rs.print("final heading: %.1f", inertialSensor.heading());

    return currLoc;
}  //makeTurn()

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
