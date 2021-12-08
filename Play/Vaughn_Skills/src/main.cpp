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

using namespace vex;

competition Competition;


void pre_auton( void ) {
  //inertial_sensor.calibrate();
}

void turnToHeadingWithSleep(vex::smartdrive& sd, double tgt, vex::rotationUnits ru, double speed, vex::velocityUnits vu)
{
  double nap = 400;

  vex::task::sleep(nap);
  sd.turnToHeading(tgt, vex::rotationUnits::deg, speed, vu);
  vex::task::sleep(nap);
}

void goStraightWithGyro(vex::directionType dt, vex::smartdrive& sd, double dist, vex::distanceUnits du, int nSteps, double speed, vex::velocityUnits vu, double tgtHeading, double extraDist)
{
  // go as far as dist by nSteps equal steps
  // need a small extra distance per *step* to compensate the stop-and-go loss
  // go "straight" by maintaining the heading at value tgtHeading

  for (int i = 0; i < 7; ++i) {
    sd.driveFor(dt, dist/i + extraDist, du, speed, vu);
    sd.turnToHeading(tgtHeading, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  }
}

void goStraight(double dist, vex::directionType dt, double tgtHeading, double speed, double kp = 0.01) {
  leftdrive.resetRotation();
  rightdrive.resetRotation();

  double distToGo = dist;

  while (distToGo > 0) {
    double headingError = inertial_sensor.heading() - tgtHeading;
    if (headingError < -270) headingError = headingError + 360;
    if (headingError > 270) headingError = headingError - 360;

    if (headingError < -15) headingError = -5;
    if (headingError > 15) headingError = 5;

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
  }
  leftdrive.stop();
  rightdrive.stop();
}

void autonomous( void ) {

  // calibrate
  inertial_sensor.calibrate();
  vex::task::sleep(2000); 

  // set up variables
  double pushSpeed = 52;
  double tileSize = 23.5;
  double rotationSpeed = 40;

  // push first alliance mogo
  leftintake.spinFor(vex::directionType::rev, 0.3, vex::rotationUnits::rev);
  lift.rotateFor(vex::directionType::rev, 50, vex::rotationUnits::deg);
  goStraight(3.5 * tileSize, vex::directionType::fwd, 0, pushSpeed);
  sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);

  // coming back from alliance mogo
  goStraight(2 * tileSize, vex::directionType::rev, 0, pushSpeed);
  turnToHeadingWithSleep(sdrive, 280, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);
  turnToHeadingWithSleep(sdrive, 270, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);
  goStraight(0.85 * tileSize, vex::directionType::fwd, 270, pushSpeed);
  turnToHeadingWithSleep(sdrive, 0, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);
  
  // push neutral mobile goal 1
  goStraight(2 * tileSize, vex::directionType::fwd, 0, pushSpeed);
  vex::task::sleep(500);
  // turnToHeadingWithSleep(sdrive, 0, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);

  // coming back from first neutral mogo
  goStraight(2 * tileSize, vex::directionType::rev, 0, pushSpeed);
  turnToHeadingWithSleep(sdrive, 280, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);
  turnToHeadingWithSleep(sdrive, 270, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);
  goStraight(1.25 * tileSize, vex::directionType::fwd, 270, pushSpeed);
  sdrive.turnToHeading(0, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);

  // push neutral mobile goal 2
  goStraight(2 * tileSize, vex::directionType::fwd, 0, pushSpeed);
  turnToHeadingWithSleep(sdrive, 0, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);

  // coming back from second neutral mogo
  goStraight(2 * tileSize, vex::directionType::rev, 0, pushSpeed);
   turnToHeadingWithSleep(sdrive, 280, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);
  turnToHeadingWithSleep(sdrive, 270, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);
  goStraight(1.3 * tileSize, vex::directionType::fwd, 270, pushSpeed);
  turnToHeadingWithSleep(sdrive, 0, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);

  // pushing third neutral mogo
  goStraight(1.5 * tileSize, vex::directionType::fwd, 0, pushSpeed);
  vex::task::sleep(100);
  goStraight(1.5 * tileSize, vex::directionType::rev, 0, pushSpeed);
  turnToHeadingWithSleep(sdrive, 45, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);
  goStraight(sqrt(2) * tileSize - 4, vex::directionType::rev, 45, pushSpeed);
  turnToHeadingWithSleep(sdrive, 90, vex::rotationUnits::deg, rotationSpeed, vex::velocityUnits::pct);
  lift.rotateFor(vex::directionType::fwd, 150, vex::rotationUnits::deg);
  goStraight(1.8 * tileSize, vex::directionType::fwd, 90, 70);
  goStraight(0.5 * tileSize, vex::directionType::fwd, 90, 20);
  goStraight(5, vex::directionType::fwd, 90, 10);
  }


void usercontrol( void ) {

  while (1) {

    if (rc.ButtonR1.pressing()) {
      lift.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonR2.pressing()) {
      lift.spin(vex::directionType::rev);
    }

    else {
      lift.stop(vex::brakeType::hold);
    }

    double leftMotorSpeed = rc.Axis3.position(vex::percentUnits::pct) * 0.7;
    double rightMotorSpeed = rc.Axis2.position(vex::percentUnits::pct) * 0.7;

    if (fabs(leftMotorSpeed) > 5.0) {
      backleftdrive.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      backleftdrive.spin(fwd);
      frontleftdrive.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      frontleftdrive.spin(fwd);
    }
    else {
      backleftdrive.stop(vex::brakeType::hold);
      frontleftdrive.stop(vex::brakeType::hold);
    }

    if (fabs(rightMotorSpeed) > 5.0) {
      backrightdrive.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      backrightdrive.spin(fwd);  
      frontrightdrive.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      frontrightdrive.spin(fwd);
    }
    else {
      backrightdrive.stop(vex::brakeType::hold);
      frontrightdrive.stop(vex::brakeType::hold);
    }

    if (rc.ButtonL2.pressing()) {
      leftintake.setVelocity(10, vex::percentUnits::pct);

      //intake.setVelocity(30, vex::percentUnits::pct);
      leftintake.spin(vex::directionType::fwd);
    }

    else if (rc.ButtonL1.pressing()) {
      
      leftintake.setVelocity(10, vex::percentUnits::pct);
      leftintake.spin(vex::directionType::rev);

    }

    else {
      leftintake.stop(vex::brakeType::hold);

    }

    if (rc.ButtonA.pressing()) {
      rightintake.setVelocity(10, vex::percentUnits::pct);
      rightintake.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonB.pressing()) {
      rightintake.setVelocity(10, vex::percentUnits::pct);
      rightintake.spin(vex::directionType::rev);
    }
    else {
      rightintake.stop(vex::brakeType::hold);
    }

    vex::task::sleep(50);
  }
}

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Brain.Screen.print("Get ready to start!");
  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton();

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}