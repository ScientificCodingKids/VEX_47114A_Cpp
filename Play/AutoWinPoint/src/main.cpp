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

using namespace vex;
using namespace std;

competition Competition;


void pre_auton( void ) {
}

void goStraight(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, double kp = 0.02, vex::brakeType bt = brake) {
  leftdrive.resetRotation();
  rightdrive.resetRotation();

  double distToGo = dist;
  double distTravelled = dist - distToGo;
  double finalSpeed = 10;
  double speed = originalSpeed;
  double const adaptiveInterval = 10;

  while (distToGo > 0) {
    double headingError = inertialSensor.heading() - tgtHeading;
    if (headingError < -270) headingError = headingError + 360;
    if (headingError > 270) headingError = headingError - 360;

    if (headingError < -15) headingError = -15;
    if (headingError > 15) headingError = 15;

    //if (dist >= 2*adaptiveInterval) {
    speed = originalSpeed;
    if (distTravelled < adaptiveInterval) speed = originalSpeed * distTravelled / adaptiveInterval;
    if (distToGo < adaptiveInterval) speed = originalSpeed * (1 - (adaptiveInterval - distToGo) / adaptiveInterval);
    //}
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
    distTravelled = dist - distToGo;
  }
  cout << "finalSpeed = " << speed << endl;
  Brain.Screen.print("finalSpeed = %f ", speed);
  leftdrive.stop(bt);
  rightdrive.stop(bt);
}

void makeTurn(double tgtHeading, bool turnClockwise, double speed=15, double kp=0.03, double tol=0.1)
{
  leftdrive.resetRotation();
  rightdrive.resetRotation();

  double degreeToGo = tgtHeading - inertialSensor.heading();

  if (degreeToGo < 0) {
    degreeToGo = degreeToGo + 360.0;
  }

  double CWDegreeToGo = degreeToGo;
  double CCWDegreeToGo = 360 - degreeToGo;


  while (degreeToGo > tol) {

    //1. compute cw, ccw degreeToGo
    CWDegreeToGo = tgtHeading - inertialSensor.heading();
    CCWDegreeToGo = 360 - CWDegreeToGo;
    
    if (CWDegreeToGo < 0) {
      CWDegreeToGo = CWDegreeToGo + 360;
      CCWDegreeToGo = 360 - CWDegreeToGo;
    }

    //2. determine rotation direction and degreeToGo
    if (CWDegreeToGo < CCWDegreeToGo) {
      degreeToGo = CWDegreeToGo;
    }
    else {
      degreeToGo = CCWDegreeToGo;
    }
    
    double headingError = degreeToGo;
    double currentSpeed = speed * kp * headingError; // when close to target heading, the speed should be low (but not 0)

    // if kp is larger, correction is greater; if kp is smaller, correction is smaller

    bool isClose = false;

    if (headingError > 15) {
      currentSpeed = speed;
    }

    else if (headingError < -15) {
      currentSpeed = speed;
    }
    else {
      isClose = true;
    }
    
    bool currentTurnClockwise = turnClockwise;

    if (isClose) {
      if (CWDegreeToGo < CCWDegreeToGo) {
        currentTurnClockwise = true; 
        degreeToGo = CWDegreeToGo;
      }
      else {
        currentTurnClockwise = false; 
        degreeToGo = CCWDegreeToGo;
      }
    }

    if (currentSpeed < 5) {
      currentSpeed = 5;
    }

    // 3. set motor speed and direction
    Brain.Screen.print("%f, %d \n", currentSpeed, isClose);
    cout << inertialSensor.heading() << ": [ " << CWDegreeToGo << ", " << CCWDegreeToGo << "]" << degreeToGo << ", " << headingError << ", " << currentSpeed << "; " << isClose << endl;  // print to terminal

   
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

    vex::task::sleep(10);
  }
  
 
  leftdrive.stop();
  rightdrive.stop();
  Brain.Screen.print("done");
 
}

void autonomous( void ) {
  inertialSensor.calibrate();
  vex::task::sleep(1500);

  double turnSpeed = 50;
  double driveSpeed = 80;

  // placing first ring in left mogo
  lift.spinFor(vex::directionType::fwd, 1.7, vex::rotationUnits::rev, 50, vex::velocityUnits::pct, false);
  vex::task::sleep(500);
  frontintake.spinFor(vex::directionType::rev, 100, vex::rotationUnits::deg, 30, vex::velocityUnits::pct, false);
  vex::task::sleep(800);
  frontintake.spinFor(vex::directionType::fwd, 100, vex::rotationUnits::deg, false);
  dt.driveFor(vex::directionType::rev, 3, vex::distanceUnits::in, driveSpeed, vex::velocityUnits::pct);

  // moving over to right mogo
  makeTurn(270, false, turnSpeed);
  goStraight(23.5, vex::directionType::fwd, 270, driveSpeed);
  makeTurn(0, true, turnSpeed);
  goStraight(3.7 * 23.5, vex::directionType::fwd, 0, 80);

  // place second ring in right mogo
  frontintake.spinFor(vex::directionType::rev, 180, vex::rotationUnits::deg, false);
  vex::task::sleep(800);
  frontintake.spinFor(vex::directionType::fwd, 180, vex::rotationUnits::deg, false);
  dt.driveFor(vex::directionType::rev, 3, vex::distanceUnits::in, driveSpeed, vex::velocityUnits::pct);
  dt.driveFor(vex::directionType::fwd, 20, vex::distanceUnits::in, driveSpeed-10, vex::velocityUnits::pct, false);

  // prep to start
  backintake.spinFor(vex::directionType::fwd, 3.75, vex::rotationUnits::rev, 80, vex::velocityUnits::pct);
  vex::task::sleep(500);
  dt.driveFor(vex::directionType::rev, 5, vex::distanceUnits::in, driveSpeed-10, vex::velocityUnits::pct, false);
  lift.spinFor(vex::directionType::rev, 1.7, vex::rotationUnits::rev, 50, vex::velocityUnits::pct, false);
}


void usercontrol( void ) {
  double liftSpeed = 70;
  bool pressIn = false;
  bool deployFork = false;
  bool breaktypebreak = false;

  backleftdrive.setBrake(vex::brakeType::coast);
  backrightdrive.setBrake(vex::brakeType::coast);
  frontleftdrive.setBrake(vex::brakeType::coast);
  frontrightdrive.setBrake(vex::brakeType::coast);

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

    if (rc.ButtonLeft.pressing()) {
      breaktypebreak = true;
      backleftdrive.setBrake(vex::brakeType::brake);
      backrightdrive.setBrake(vex::brakeType::brake);
      frontleftdrive.setBrake(vex::brakeType::brake);
      frontrightdrive.setBrake(vex::brakeType::brake);
    }
    if (rc.ButtonUp.pressing()) {
      breaktypebreak = false;
      backleftdrive.setBrake(vex::brakeType::coast);
      backrightdrive.setBrake(vex::brakeType::coast);
      frontleftdrive.setBrake(vex::brakeType::coast);
      frontrightdrive.setBrake(vex::brakeType::coast);
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
      backleftdrive.stop();
      frontleftdrive.stop();
    }

    if (fabs(rightMotorSpeed) > 5.0) {
      backrightdrive.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      backrightdrive.spin(fwd);  
      frontrightdrive.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      frontrightdrive.spin(fwd);
    }
    else {
      if (rc.ButtonLeft.pressing()) {
        backrightdrive.stop();
        frontrightdrive.stop();
      }
      else {
      backrightdrive.stop();
      frontrightdrive.stop();
      }
    }

    if (rc.ButtonDown.pressing()) {
      deployFork = true;
    }
    if (deployFork == true) {
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

    if (pressIn == true) {
      frontintake.setVelocity(10, vex::percentUnits::pct);
      frontintake.spin(vex::directionType::rev);
    }


    vex::task::sleep(50);
  } // while loop
} // usercontrol

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