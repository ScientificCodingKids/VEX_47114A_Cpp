#ifndef _motion_hpp_
#define _motion_hpp_

// #include "vex.h"  -- SHOULD NOT DEPEND ON A SPECIFIC PROJECT!!!
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>
#include <cassert>

#include "nutils.hpp"
using namespace vex;
using namespace std;


class DriveTrainBase {
    public:
    DriveTrainBase(vex::motor& bl, vex::motor& br, vex::motor& fl, vex::motor& fr, vex::brain& brn,  vex::inertial& ins): 
        backleftdrive(bl), backrightdrive(br), frontleftdrive(fl), frontrightdrive(fr), _brain(brn), inertialSensor(ins) {
            ;
    }
    vex::motor& backleftdrive;
    vex::motor& backrightdrive;
    vex::motor& frontleftdrive;
    vex::motor& frontrightdrive;

    vex::brain& _brain;
    vex::inertial& inertialSensor;

    RollingScreen* _rollScr; // why not initialized in constructor()?

    void setRollingScreen(RollingScreen* rs) { _rollScr = rs; }

    void calibrate () {
      int v = 0;  // get around syntax of print()
      _rollScr->setPenColor(vex::color::red);

      _rollScr->print("START calib: %d", v);

      inertialSensor.calibrate();

      while (inertialSensor.isCalibrating()) {
        // busy waiting
        vex::task::sleep(50);
      }

      _rollScr->print("END calib: %d", v);     
      _rollScr->setPenColor(vex::color::black);

    }

  Coord goStraight(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, double kp = 0.02, vex::brakeType bt = brake, Coord srcLoc = Coord(0.0, 0.0));

  Coord makeTurn(double tgtHeading, bool turnClockwise, double speed=15, double kp=0.03, double tol=1, Coord srcLoc = Coord(0.0, 0.0));

  Coord gotoCoord(double startX, double startY, double destX, double destY, double originalSpeed);
};  // class DriveTrainBase


Coord DriveTrainBase::goStraight(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, double kp, vex::brakeType bt, Coord srcLoc) {
  Coord currLoc = srcLoc;

  leftdrive.resetPosition();
  rightdrive.resetPosition();

  double distToGo = dist; // distance more to travel
  double distTravelled = dist - distToGo; // distance already traveled
  double finalSpeed = 10; // capping speed
  double speed = originalSpeed; // max speed
  double const adaptiveInterval = 10; // slow down/speed up interval
  double prevRot = leftdrive.position(vex::rotationUnits::deg); // amount of rotation at the last run through
  double changedRotations = leftdrive.position(vex::rotationUnits::deg) - prevRot; // rotations passed since last loop
  double dx = 0; // change in x coordinate since last loop
  double dy = 0; // change in y coordinate since last loop
 
  while (distToGo > 0) {
    double headingError = inertialSensor.heading() - tgtHeading;
    if (headingError < -270) headingError = headingError + 360;
    if (headingError > 270) headingError = headingError - 360;

    if (headingError < -15) headingError = -15;
    if (headingError > 15) headingError = 15;

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

    distToGo = dist - fabs(leftdrive.position(vex::rotationUnits::deg) / 360 * (4.0 * 3.1415269265));

    changedRotations = (leftdrive.position(vex::rotationUnits::deg) * 4.0 * 3.1415269265) / 360 - prevRot; // need scale, deg => inch

    dx = changedRotations * cos(90-inertialSensor.heading());
    dy = changedRotations * sin(90-inertialSensor.heading());

    prevRot = (leftdrive.position(vex::rotationUnits::deg) * 4.0 * 3.1415269265) / 360;

    currLoc.x = dx + currLoc.x;
    currLoc.y = dy + currLoc.y;
    
    distTravelled = dist - distToGo;
  }
  cout << "finalSpeed = " << speed << endl;
  _brain.Screen.print("finalSpeed = %f ", speed);
  leftdrive.stop(bt);
  rightdrive.stop(bt);
  cout << "ending coordinate = " << currLoc.x << ", " << currLoc.y << endl;
  _brain.Screen.print("ending coordinate = %f, %f", currLoc.x, currLoc.y);
  return currLoc;
}  // DriveTrainBase::goStraight()

 
Coord DriveTrainBase::makeTurn(double tgtHeading, bool turnClockwise, double speed, double kp, double tol, Coord srcLoc)
{
  // using straight line method to measure
  // close enough to arc (within 0.1)
  // code for arc is commented out

  RollingScreen rs(_brain.Screen);

  Coord currLoc = srcLoc;
  //Coord currLoc2 = srcLoc;

  leftdrive.resetPosition();
  rightdrive.resetPosition();

  double degreeToGo = tgtHeading - inertialSensor.heading();

  if (degreeToGo < 0) {
    degreeToGo = degreeToGo + 360.0;
  }

  double CWDegreeToGo = degreeToGo;
  double CCWDegreeToGo = 360 - degreeToGo;

  double lrot = leftdrive.position(vex::rotationUnits::deg);
  double prevDegree = 0.0;
  double changedRotations = lrot - prevDegree; // rotations passed since last loop
  double dx = 0;
  double dy = 0;
  double pastTravelled = 0; // inches
  //double radius = 0;
  //double dx2 = 0;
  //double dy2 = 0;
  double chOld = 0;
  double currentTurnClockwise = turnClockwise;

  while (degreeToGo > tol) {
    double travelledDist = rotation2distance(leftdrive.position(vex::rotationUnits::deg)) - pastTravelled;
    double ch = inertialSensor.heading();
    double chArc = degree2arc(ch);
    //double chOldArc = degree2arc(chOld);
    lrot = leftdrive.position(vex::rotationUnits::deg);

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
    degreeToGo = CWDegreeToGo < CCWDegreeToGo ? CWDegreeToGo : CCWDegreeToGo;
    
    double headingError = degreeToGo;
    double currentSpeed = speed * kp * headingError; // when close to target heading, the speed should be low (but not 0)

    // if kp is larger, correction is greater; if kp is smaller, correction is smaller

    bool isClose = false;

    if (headingError > 15 || headingError < -15) {
      currentSpeed = speed;
    }

    else {
      currentSpeed = speed*(1-(15 - degreeToGo)/15);
      isClose = true;
    }
    
    if (isClose) {
      currentTurnClockwise = CWDegreeToGo < CCWDegreeToGo;
    }
    else {
      currentTurnClockwise = turnClockwise;
    }

    if (currentSpeed < 5) {
      currentSpeed = 5;
    }

    // 3. set motor speed and direction
    //_brain.Screen.print("%f, %d \n", currentSpeed, isClose);
    //cout << ch << ": [ " << CWDegreeToGo << ", " << CCWDegreeToGo << "]" << degreeToGo << ", " << headingError << ", " << currentSpeed << "; " << isClose << "; " << currentTurnClockwise << endl;  // print to terminal

   
    leftdrive.setVelocity(currentSpeed, vex::percentUnits::pct);
    rightdrive.setVelocity(currentSpeed, vex::velocityUnits::pct);

    if (currentTurnClockwise) {
      leftdrive.spin(vex::directionType::fwd);
      rightdrive.spin(vex::directionType::rev);
    }
    else {
      leftdrive.spin(vex::directionType::rev);
      rightdrive.spin(vex::directionType::fwd);
    }


	//update coordinates

    changedRotations = lrot - prevDegree;
    //if (ch - chOld == 0) radius = 0;
    //else radius = abs(travelledDist)/degree2arc(ch - chOld);

    dx = abs(travelledDist) * sin(chArc);
    dy = abs(travelledDist) * cos(chArc);

    //dx2 = radius * -cos(chArc) + radius * cos(chOldArc); 
    //dy2 = radius * sin(chArc) - radius * sin(chOldArc);

   
    currLoc.x = dx + currLoc.x;
    currLoc.y = dy + currLoc.y;

    //currLoc2.x = dx2 + currLoc2.x;
    //currLoc2.y = dy2 + currLoc2.y;

    pastTravelled = rotation2distance(leftdrive.position(vex::rotationUnits::deg));

    prevDegree = lrot;

    chOld = ch;
    vex::task::sleep(10);
  }
  
  leftdrive.stop();
  rightdrive.stop();
  // SmartScreen ss(Brain.Screen, 1, 2);
  // ss.printAt(1, "straight line location: (%.2f, %.2f)", currLoc.x, currLoc.y);
 
  return currLoc;
}    // DriveTrainBase::makeTurn()


Coord DriveTrainBase::gotoCoord(double startX, double startY, double destX, double destY, double originalSpeed) {
  // WARN: this function is NOT tested and NOT used [yet]
  leftdrive.resetPosition();
  rightdrive.resetPosition();

  // length of travel
  double xToGo = startX-destX;
  double yToGo = startY-destY;
  double distToGo = sqrt(xToGo * xToGo + yToGo * destY);
  double quadrant = 0;
  // angle of travel
  double triangleAngle = asin(xToGo / distToGo);
  double angle = triangleAngle;
  if (destX >= startX) {
    if (destY >= startY) {
      quadrant = 1;
      angle = triangleAngle;
    }
    else {
      quadrant = 4;
      angle = 90 + triangleAngle;
    }
  }
  else {
    if (destY >= startY) {
      quadrant = 2;
      angle = 360 - triangleAngle;
    }
    else {
      quadrant = 3;
      angle = 180 + triangleAngle;
    }
  }

  Coord srcLoc(startX, startY);

  Coord loc1 = makeTurn(angle, true, originalSpeed, 0.03, 1.0, srcLoc); // make sure angle is heading

  return goStraight(distToGo, vex::directionType::fwd, angle, originalSpeed, 0.02, vex::brake, loc1);
}  // DriveTrainBase::gotoCoord()


#endif  // _motion_hpp_
