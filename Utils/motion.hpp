//#include "C:/Program Files (x86)/VEX Robotics/VEXcode Pro V5/sdk/vexv5/include/v5_color.h"
#include "vex.h"
#include <iostream>
#include <cmath>
#include <cassert>

#include "nutils.hpp"
using namespace vex;
using namespace std;


class Coord {
  public:
    double x, y;

    Coord(double x0, double y0): x(x0), y(y0) {;}
};

double rotation2distance(double deg, double gearRatio = 1, double wheelDiameter = 4.15) {
  // returns distance in inches
  double distance = (gearRatio * deg * wheelDiameter * M_PI) / 360;
  return distance;
}

double degree2arc(double deg) {
  return deg * M_PI / 180;
}

double arc2deg(double arc) {
  return 180 * M_PI / arc;
}



Coord goStraight(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, double kp = 0.02, vex::brakeType bt = brake, Coord srcLoc = Coord(0.0, 0.0)) {
  Coord currLoc = srcLoc;

  leftdrive.resetRotation();
  rightdrive.resetRotation();

  double distToGo = dist; // distance more to travel
  double distTravelled = dist - distToGo; // distance already traveled
  double finalSpeed = 10; // capping speed
  double speed = originalSpeed; // max speed
  double const adaptiveInterval = 10; // slow down/speed up interval
  double prevRot = leftdrive.rotation(vex::rotationUnits::deg); // amount of rotation at the last run through
  double changedRotations = leftdrive.rotation(vex::rotationUnits::deg) - prevRot; // rotations passed since last loop
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
 
void goPlatform(double initialSpeed=60) {
  double currentRoll = inertialSensor.roll(); // roll (angle) of robot
  bool goforward = false; // direction
  bool isDone = false; // exit while loop
  double k = 1.0; // coefficient on the speed
  double speed = initialSpeed; // adjusted speed
  bool isRampUp = true; // whether it is the first time going up the ramp
  double counter = 0;

  while ((abs(currentRoll) >= 1 || isRampUp) && ~isDone) {
    currentRoll = inertialSensor.roll();
    speed = k * currentRoll;
    if (counter == 3) {
    counter = 0;
    }
    counter = counter + 1;
    // when to turn off isRampUp
    if (currentRoll > 20) {
      isRampUp = false;
    }
    // when to ignore k, revert to original
    if ((speed > initialSpeed) || (isRampUp)) {
      speed = initialSpeed;
    }


    if (abs(currentRoll) < 20 && ~isRampUp) {
      speed = 5;
    }
    else {
      speed = 45;
    }

    if (speed>(initialSpeed-5) && (counter==1 || counter==2)) {
        speed = 0;
    }

    // changing directions
    if (currentRoll >= 0) {
      goforward = true;
      dt.drive(vex::directionType::fwd, speed, vex::velocityUnits::pct);
    }
    else if (currentRoll < 0) {
      if (~isRampUp) {
       dt.drive(vex::directionType::rev, speed, vex::velocityUnits::pct);
      }
      if (isRampUp) {
       dt.drive(vex::directionType::fwd, speed, vex::velocityUnits::pct);
      }
    }
    if (rc.ButtonX.pressing()) {
      isDone = true;
    }
    vex::task::sleep(10);
  }

  dt.stop(vex::brakeType::hold);
}

void goPlatformWithRotation(double initialSpeed=60, double slowSpeed = 20, double tgtHeading = 0) {
  leftdrive.resetRotation();
  rightdrive.resetRotation();

  double currentRoll = inertialSensor.roll(); // roll (angle) of robot
  bool goforward = false; // direction
  bool isDone = false; // exit while loop
  double kp = 0.02;
  double speed = initialSpeed; // adjusted speed
  bool isRampUp = true; // whether it is the first time going up the ramp
  double headingError = inertialSensor.heading() - tgtHeading;
  double wheelRotationIn = (leftdrive.rotation(vex::rotationUnits::deg) * 4.0 * 3.1415269265) / 360;

  while ((abs(currentRoll) >= 1 || isRampUp) && ~isDone) {
    currentRoll = inertialSensor.roll();
    wheelRotationIn = (leftdrive.rotation(vex::rotationUnits::deg) * 4.0 * 3.1415269265) / 360;

    if ((wheelRotationIn < 23.5) && (isRampUp)) {
      speed = initialSpeed;
    }
    else if ((wheelRotationIn > 23.5) || (~isRampUp)) {
      speed = slowSpeed;
    }
    // changing directions
    if (currentRoll >= 0) {
      goforward = true;
    }
    else if (currentRoll < 0) {
      if (~isRampUp) {
       goforward = false;
      }
      if (isRampUp) {
       goforward = true;
      }
    }

    // adding goStraight

    headingError = inertialSensor.heading() - tgtHeading;
    if (headingError < -270) headingError = headingError + 360;
    if (headingError > 270) headingError = headingError - 360;

    if (headingError < -15) headingError = -15;
    if (headingError > 15) headingError = 15;

    if (goforward) {
      leftdrive.setVelocity(speed * (1 - kp * headingError), vex::percentUnits::pct);
      rightdrive.setVelocity(speed * (1 + kp * headingError), vex::percentUnits::pct);
      leftdrive.spin(vex::directionType::fwd);
      rightdrive.spin(vex::directionType::fwd);
    } else {
      leftdrive.setVelocity(speed * (1 + kp * headingError), vex::percentUnits::pct);
      rightdrive.setVelocity(speed * (1 - kp * headingError), vex::percentUnits::pct);
      leftdrive.spin(vex::directionType::rev);
      rightdrive.spin(vex::directionType::rev);
    }

    if (rc.ButtonX.pressing()) {
      isDone = true;
    }
    vex::task::sleep(10);
  }

  dt.stop(vex::brakeType::hold);
}


Coord makeTurn(double tgtHeading, bool turnClockwise, double speed=15, double kp=0.03, double tol=1, Coord srcLoc = Coord(0.0, 0.0))
{
  RollingScreen rs(Brain.Screen);

  Coord currLoc = srcLoc;
  Coord currLoc2 = srcLoc;

  leftdrive.resetRotation();
  rightdrive.resetRotation();

  double degreeToGo = tgtHeading - inertialSensor.heading();

  if (degreeToGo < 0) {
    degreeToGo = degreeToGo + 360.0;
  }

  double CWDegreeToGo = degreeToGo;
  double CCWDegreeToGo = 360 - degreeToGo;

  double lrot = leftdrive.rotation(vex::rotationUnits::deg);
  double prevDegree = 0.0;
  double changedRotations = lrot - prevDegree; // rotations passed since last loop
  double dx = 0;
  double dy = 0;
  double pastTravelled = 0; // inches
  double radius = 0;
  double dx2 = 0;
  double dy2 = 0;
  double chOld = 0;
  double currentTurnClockwise = turnClockwise;

  while (degreeToGo > tol) {
    double travelledDist = rotation2distance(leftdrive.rotation(vex::rotationUnits::deg)) - pastTravelled;
    double ch = inertialSensor.heading();
    double chArc = degree2arc(ch);
    double chOldArc = degree2arc(chOld);
    lrot = leftdrive.rotation(vex::rotationUnits::deg);

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
    //Brain.Screen.print("%f, %d \n", currentSpeed, isClose);
    //cout << ch << ": [ " << CWDegreeToGo << ", " << CCWDegreeToGo << "]" << degreeToGo << ", " << headingError << ", " << currentSpeed << "; " << isClose << "; " << currentTurnClockwise << endl;  // print to terminal

   
    leftdrive.setVelocity(currentSpeed, vex::percentUnits::pct);
    rightdrive.setVelocity(currentSpeed, vex::velocityUnits::pct);

    if (currentTurnClockwise) {
      leftdrive.spin(vex::directionType::fwd);
      rightdrive.stop(vex::brakeType::coast);
      rightdrive.spin(vex::directionType::rev);
    }
    else {
      leftdrive.spin(vex::directionType::rev);
      rightdrive.spin(vex::directionType::fwd);
      rightdrive.stop(vex::brakeType::coast);
    }


	//update coordinates

    changedRotations = lrot - prevDegree;
    if (ch - chOld == 0) radius = 0;
    else radius = travelledDist/degree2arc(ch - chOld);

    dx = travelledDist * sin(chArc);
    dy = travelledDist * cos(chArc);

    dx2 = radius * -cos(chArc) + radius * cos(chOldArc);
    dy2 = radius * sin(chArc) + radius * sin(chOldArc);

   
    currLoc.x = dx + currLoc.x;
    currLoc.y = dy + currLoc.y;

    currLoc2.x = dx2 + currLoc2.x;
    currLoc2.y = dy2 + currLoc2.y;

    pastTravelled = rotation2distance(leftdrive.rotation(vex::rotationUnits::deg));

    prevDegree = lrot;

    chOld = ch;
    vex::task::sleep(10);
  }
  
  leftdrive.stop();
  rightdrive.stop();
  // Brain.Screen.print("straight line currLoc: %f, %f", currLoc.x, currLoc.y);
  Brain.Screen.print("arc currLoc: %f, %f", currLoc2.x, currLoc2.y);
 
  return currLoc2;
}

void gotoCoord(double startX, double startY, double destX, double destY, double originalSpeed) {
  leftdrive.resetRotation();
  rightdrive.resetRotation();

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
  makeTurn(angle, true, originalSpeed); // make sure angle is heading
  goStraight(distToGo, vex::directionType::fwd, angle, originalSpeed);
}

