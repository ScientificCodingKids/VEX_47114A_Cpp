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

Coord goStraightnew(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, double kp = 0.02, vex::brakeType bt = brake, Coord srcLoc = Coord(0.0, 0.0)) {
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

    vex::task::sleep(50);

    distToGo = dist - rotation2distance(fabs(leftdrive.rotation(vex::rotationUnits::deg)));

    changedRotations = rotation2distance(leftdrive.rotation(vex::rotationUnits::deg)) - prevRot; // need scale, deg => inch

    dx = changedRotations * sin(degree2arc(inertialSensor.heading()));
    dy = changedRotations * cos(degree2arc(inertialSensor.heading()));

    prevRot = rotation2distance(leftdrive.rotation(vex::rotationUnits::deg));

    currLoc.x = dx + currLoc.x;
    currLoc.y = dy + currLoc.y;
    
    distTravelled = dist - distToGo;
    cout << inertialSensor.heading() << ", " << changedRotations << ", " << dx << ", " << dy << ", " << currLoc.x << ", " << currLoc.y << endl;
  }
  //cout << "finalSpeed = " << speed << endl;
  //Brain.Screen.print("finalSpeed = %f ", speed);
  leftdrive.stop(bt);
  rightdrive.stop(bt);
  //cout << "ending coordinate = " << currLoc.x << ", " << currLoc.y << endl;
  //Brain.Screen.print("ending coordinate = %f, %f", currLoc.x, currLoc.y);
  return currLoc;
} 


Coord gotoCoordnew(double startX, double startY, double destX, double destY, double originalSpeed) {
  leftdrive.resetRotation();
  rightdrive.resetRotation();

  // length of travel
  double xToGo = destX - startX;
  double yToGo = destY - startY;
  double distToGo = sqrt(xToGo * xToGo + yToGo * yToGo);
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
  // makeTurn(angle, true, originalSpeed); // make sure angle is heading
  Coord endCoord = goStraightnew(distToGo, vex::directionType::fwd, angle, originalSpeed);
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


void pre_auton( void ) {
}

void autonomous( void ) {
  inertialSensor.calibrate();
  vex::task::sleep(1500);
  Coord printCoord = goStraightnew(10, vex::directionType::fwd, 0, 70);
  // Coord printCoord = gotoCoordnew(0, 0, 50, 50, 70);
  //Brain.Screen.clearScreen();
  Brain.Screen.print("location: %4.1f, %4.1f", printCoord.x, printCoord.y);
}

void usercontrol( void ) {
  double liftSpeed = 70;
  bool pressIn = false;
  bool deployFork = false;
  bool runPlatform = false;

  while (1) {

    if (rc.ButtonR1.pressing()) {
      lift.setVelocity(liftSpeed, vex::percentUnits::pct);
      lift.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonR2.pressing()) {
      lift.setVelocity(liftSpeed+10, vex::percentUnits::pct);
      lift.spin(vex::directionType::rev);
    }

    else {
      lift.stop(vex::brakeType::hold);
    }

    if (rc.ButtonUp.pressing()) {
      runPlatform = true;
    }
    if (runPlatform) {
      goPlatform();
      runPlatform = false;
    }

    double leftMotorSpeed = rc.Axis3.position(vex::percentUnits::pct) * 0.85;
    double rightMotorSpeed = rc.Axis2.position(vex::percentUnits::pct) * 0.85;

    if (fabs(leftMotorSpeed) > 5.0) {
      backleftdrive.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      backleftdrive.spin(fwd);
      frontleftdrive.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      frontleftdrive.spin(fwd);
    }
    else {
      if (rc.ButtonLeft.pressing()) {
        backleftdrive.stop(vex::brakeType::brake);
        frontleftdrive.stop(vex::brakeType::brake);
      }
      else {
      backleftdrive.stop(vex::brakeType::coast);
      frontleftdrive.stop(vex::brakeType::coast);
      }
    }

    if (fabs(rightMotorSpeed) > 5.0) {
      backrightdrive.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      backrightdrive.spin(fwd);  
      frontrightdrive.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      frontrightdrive.spin(fwd);
    }
    else {
      if (rc.ButtonLeft.pressing()) {
        backrightdrive.stop(vex::brakeType::brake);
        frontrightdrive.stop(vex::brakeType::brake);
      }
      else {
      backrightdrive.stop(vex::brakeType::coast);
      frontrightdrive.stop(vex::brakeType::coast);
      }
    }

    if (rc.ButtonDown.pressing()) {
      deployFork = true;
    }
    if (deployFork) {
      backintake.spinFor(vex::directionType::fwd, 3.75, vex::rotationUnits::rev, 80, vex::velocityUnits::pct);
      deployFork = false;
    }

    if (rc.ButtonL2.pressing()) {
      backintake.setVelocity(70, vex::percentUnits::pct);
      backintake.spin(vex::directionType::fwd);
    }

    else if (rc.ButtonL1.pressing()) {
      
      backintake.setVelocity(50, vex::percentUnits::pct);
      backintake.spin(vex::directionType::rev);

    }

    else {
      backintake.stop(vex::brakeType::hold);

    }

    if (rc.ButtonA.pressing()) {
      pressIn = false;
      frontintake.setVelocity(50, vex::percentUnits::pct);
      frontintake.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonB.pressing()) {
      pressIn = false;
      frontintake.setVelocity(50, vex::percentUnits::pct);
      frontintake.spin(vex::directionType::rev);
    }
    else {
      frontintake.stop(hold);
    }

    if (rc.ButtonY.pressing()) {
      pressIn = true;
    }

    if (pressIn) {
      frontintake.setVelocity(10, vex::percentUnits::pct);
      frontintake.spin(vex::directionType::rev);
    }


    vex::task::sleep(50);
  } // while loop
} // usercontrol

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  //Brain.Screen.print("Get ready to start!");
  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton();

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}