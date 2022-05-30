/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Chloe Ning                                       */
/*    Created:      Wed Jan 22 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------xsssssssssss------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Drivetrain           drivetrain    10, 16, 1, 15   
// lift                 motor_group   4, 9            
// mogo1                motor         18              
// mogo2                motor         20              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>
#include <cmath>
#include "../../../Utils/motion.hpp"
#include <cassert>
#include "../../../Utils/nutils.hpp"

using namespace vex;
using namespace std;

competition Competition;


double rotation2distance(double deg, double gearRatio = 1, double wheelDiameter = 4) {
  // returns distance in inches
  double distance = (gearRatio * deg * wheelDiameter * M_PI) / 360;
  return distance;
}

double degree2arc(double deg) {
  return deg * M_PI / 180;
}

Coord goStraightnew(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, Coord srcLoc = Coord(0.0, 0.0), double kp = 0.02, vex::brakeType bt = brake) {
  Coord currLoc = srcLoc;

  leftdrive.resetRotation();
  rightdrive.resetRotation();

  double distToGo = dist; // distance more to travel
  double distTravelled = 0; // distance already traveled
  double finalSpeed = 10; // capping speed
  double speed = originalSpeed; // max speed
  double const adaptiveInterval = 10; // slow down/speed up interval
  double prevDist = 0; // amount of rotation at the last run through
  double dx = 0; // change in x coordinate since last loop
  double dy = 0; // change in y coordinate since last loop
 
  while (distToGo > 0) {
    double ch = inertialSensor.heading();
    double headingError = ch - tgtHeading;
    double motorRot = leftdrive.rotation(vex::rotationUnits::deg);
    distTravelled = rotation2distance(fabs(motorRot));
    distToGo = dist - distTravelled;
    double distChanged = distTravelled - prevDist;

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

    vex::task::sleep(50);

    distChanged = distTravelled - prevDist;

    dx = distChanged * sin(degree2arc(ch));
    dy = distChanged * cos(degree2arc(ch));

    // change in value = distance newly travelled x sin or cos (arc of current inertial heading)

    prevDist = distTravelled;

    currLoc.x = dx + currLoc.x;
    currLoc.y = dy + currLoc.y;
    
  } // while loop
  //cout << "finalSpeed = " << speed << endl;
  //Brain.Screen.print("finalSpeed = %f ", speed);
  leftdrive.stop(bt);
  rightdrive.stop(bt);
  //cout << "ending coordinate = " << currLoc.x << ", " << currLoc.y << endl;
  //Brain.Screen.print("ending coordinate = %f, %f", currLoc.x, currLoc.y);
  return currLoc;
} // goStraightnew

double arc2deg(double arc) {
  return 180 * M_PI / arc;
}

Coord makeTurnnew(double tgtHeading, bool turnClockwise, double speed=15, double kp=0.03, double tol=1, Coord srcLoc = Coord(0.0, 0.0))
{

  Coord currLoc = srcLoc;

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

  while (degreeToGo > tol) {
    double travelledDist = rotation2distance(leftdrive.rotation(vex::rotationUnits::deg)) - pastTravelled;
    double ch = inertialSensor.heading();
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
    
    bool currentTurnClockwise = CWDegreeToGo < CCWDegreeToGo;

    if (currentSpeed < 5) {
      currentSpeed = 5;
    }

    // 3. set motor speed and direction
    //Brain.Screen.print("%f, %d \n", currentSpeed, isClose);
    cout << ch << ": [ " << CWDegreeToGo << ", " << CCWDegreeToGo << "]" << degreeToGo << ", " << headingError << ", " << currentSpeed << "; " << isClose << "; " << currentTurnClockwise << endl;  // print to terminal
  
   
    leftdrive.setVelocity(currentSpeed, vex::percentUnits::pct);
    rightdrive.setVelocity(currentSpeed, vex::velocityUnits::pct);

    if (currentTurnClockwise) {
      leftdrive.spin(vex::directionType::fwd);
    //  rightdrive.spin(vex::directionType::rev);
    }
    else {
      leftdrive.spin(vex::directionType::rev);
    //  rightdrive.spin(vex::directionType::fwd);
    }


	//update coordinates

    changedRotations = lrot - prevDegree;

    dx = travelledDist * sin(degree2arc(ch));
    dy = travelledDist * cos(degree2arc(ch));

    
    // change in value = distance newly travelled x sin or cos (arc of current inertial heading)
 
    currLoc.x = dx + currLoc.x;
    currLoc.y = dy + currLoc.y;

    pastTravelled = rotation2distance(leftdrive.rotation(vex::rotationUnits::deg));

    vex::task::sleep(10);
  }
  
  leftdrive.stop();
  rightdrive.stop();
  Brain.Screen.print("done");
 
  return currLoc;
}


Coord gotoCoordnew(Coord startLoc, Coord destLoc, double originalSpeed) {

  // length of travel
  double xToGo = destLoc.x - startLoc.x;
  double yToGo = destLoc.y - startLoc.y;
  double distToGo = sqrt(xToGo * xToGo + yToGo * yToGo);
  double quadrant = 0;

  // angle of travel
  double triangleAngle = arc2deg(asin(xToGo / distToGo));
  double heading = triangleAngle;
  if (destLoc.x >= startLoc.x) {
    if (destLoc.y >= startLoc.y) {
      quadrant = 1;
      heading = triangleAngle;
    }
    else {
      quadrant = 4;
      heading = 180 - triangleAngle;
    }
  }
  else {
    if (destLoc.y >= startLoc.y) {
      quadrant = 2;
      heading = 360 - triangleAngle;
    }
    else {
      quadrant = 3;
      heading = 180 + triangleAngle;
    }
  }
  // makeTurn(angle, true, originalSpeed); // make sure angle is heading
  Coord endCoord = goStraightnew(distToGo, vex::directionType::fwd, heading, originalSpeed, startLoc);
  return endCoord;
}

void goPlatformWithRotation2(double initialSpeed=60, double slowSpeed = 20, double tgtHeading = 0) {
  leftdrive.resetRotation();
  rightdrive.resetRotation();

  inertialSensor.calibrate();
  vex::task::sleep(1500);

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

void inertialTest (void) {
  while (1) {
    cout << inertialSensor.heading() << endl;
    vex::task::sleep(1000);
  }
}

void primitiveTurnNinety(double speed) {
  leftdrive.resetRotation();
  rightdrive.resetRotation();
  inertialSensor.resetHeading();

  double degreeToGo = 90;
  double CWDegreeToGo = 90;
  double CCWDegreeToGo = 360 - CWDegreeToGo;
  double ch = inertialSensor.heading();
  bool turnClockwise = true;
  bool isClose = false;
  double currentSpeed = speed;

  while (ch < 90) {
    if (CWDegreeToGo < CCWDegreeToGo) {
      turnClockwise = true;
      degreeToGo = CWDegreeToGo;}
    else {
      turnClockwise = false;
      degreeToGo = CCWDegreeToGo;
    }

    if (degreeToGo > 15) {
      isClose = false;
      currentSpeed = speed;
    }
    else {
      isClose = true;
      currentSpeed = currentSpeed = speed*(1-(15 - degreeToGo)/15);
    }

    if (currentSpeed < 5) {
      currentSpeed = 5;
    }

    // 3. set motor speed and direction
    //Brain.Screen.print("%f, %d \n", currentSpeed, isClose);
    cout << ch << ": [ " << CWDegreeToGo << ", " << CCWDegreeToGo << "]" << degreeToGo << endl;  // print to terminal

    leftdrive.setVelocity(currentSpeed, vex::percentUnits::pct);
    rightdrive.setVelocity(currentSpeed, vex::velocityUnits::pct);

    if (turnClockwise) {
      leftdrive.spin(vex::directionType::fwd);
      rightdrive.spin(vex::directionType::rev);
    }
    else {
      leftdrive.spin(vex::directionType::rev);
      rightdrive.spin(vex::directionType::fwd);
    }

    ch = inertialSensor.heading();
    degreeToGo = 90 - rotation2distance(leftdrive.rotation(vex::rotationUnits::deg));
    CWDegreeToGo = 90 - ch;
    CCWDegreeToGo = 360 - CWDegreeToGo;
    vex::task::sleep(100);
    
  }
  leftdrive.stop();
  rightdrive.stop();
}


void pre_auton( void ) {
}

void autonomous( void ) {
  inertialSensor.calibrate();
  vex::task::sleep(1500);
  Coord printCoord = makeTurnnew(90, true, 50);
  Brain.Screen.print("%3.2f, %3.2f", printCoord.x, printCoord.y);
}

void usercontrol( void ) {
  inertialSensor.calibrate();
  vex::task::sleep(500);

  Coord actualDestLoc = makeTurnnew(90, true);
  //goStraightnew(30, vex::directionType::fwd, 0, 50);
  SmartScreen ss(Brain.Screen, 7, 8);
  ss.printAt(7, "actual loc: (%.2f, %.2f)",actualDestLoc.x, actualDestLoc.y);

  // actualDestLoc = makeTurnnew(90, true, 30, 0.03, 0.1, actualDestLoc);
  // ss.printAt(5, "actual loc: (%.2f, %.2f)",actualDestLoc.x, actualDestLoc.y);


  // Brain.Screen.setCursor(4, 1);  // has to set this each time calling print()
  // Brain.Screen.setPenColor(color(0, 0, 250));
  // Brain.Screen.clearLine(4);
  // Brain.Screen.print("end coordinates: %d, %d", printValues.x, printValues.y);
  
} // usercontrol

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Brain.Screen.setCursor(1, 1);
  // Brain.Screen.print("Built at: %s %s", __DATE__, __TIME__);

  // Brain.Screen.setCursor(3, 1);
  // Brain.Screen.print("3rd line");

  // for (int i=10; i>0; --i) {
  //   Brain.Screen.setCursor(2, 1);  // has to set this each time calling print()
  //   Brain.Screen.setPenColor(color(0, 255, 0));
  //   Brain.Screen.clearLine(2);
  //   Brain.Screen.print("count down: %d", i);
  //   task::sleep(50);
  // }


  // SmartScreen scr1(Brain.Screen, 1, 5);
  // scr1.printAt(1, "Built at: %s %s", __DATE__, __TIME__); // shows when code was compiled

  // RollingScreen scr2(Brain.Screen, 10, 5);

  // scr1.printAt(3, "Foo");

  // for (int i=100; i>0; --i) {
  //   scr2.setPenColor(color(255, 0, 0));
  //   scr2.print("roll up: %d", i);
  //   task::sleep(100);
  // }

  //Brain.Screen.print("Get ready to start!");

  inertialSensor.calibrate();
  vex::task::sleep(2500);

  Coord actualDestLoc = makeTurnnew(180, true);
  SmartScreen ss(Brain.Screen, 1, 6);
  ss.printAt(4, "actual loc: (%.2f, %.2f)",actualDestLoc.x, actualDestLoc.y);

  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton();

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}