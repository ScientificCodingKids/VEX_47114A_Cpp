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

void turnToHeadingWithSleep(vex::smartdrive& sd, double tgt, vex::rotationUnits ru, double speed, vex::velocityUnits vu)
{
  double nap = 300;

  vex::task::sleep(nap);
  sd.turnToHeading(tgt, vex::rotationUnits::deg, speed, vu);
  vex::task::sleep(nap);
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

void makeTurn(double tgtHeading, bool turnClockwise, double speed=15, double kp=0.009, double tol=0.1)
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

    if (headingError < 15) {
      headingError = 15;
      currentSpeed = speed;
      isClose = true;
    }

    if (headingError > -15) {
      headingError = -15;
      currentSpeed = speed;
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

void goSquare(int N, double a, double speed, double kp) {
  for (int i = 0; i < N; ++i) {
      goStraight(a, vex::directionType::fwd, 0, speed, kp);
      sdrive.turnToHeading(90, vex::rotationUnits::deg, speed, vex::velocityUnits::pct);

      goStraight(a, vex::directionType::fwd, 90, speed, kp);
      sdrive.turnToHeading(180, vex::rotationUnits::deg, speed, vex::velocityUnits::pct);
      
      goStraight(a, vex::directionType::fwd, 180, speed, kp);
      sdrive.turnToHeading(270, vex::rotationUnits::deg, speed, vex::velocityUnits::pct);
      
      goStraight(a, vex::directionType::fwd, 270, speed, kp);
      sdrive.turnToHeading(0, vex::rotationUnits::deg, speed, vex::velocityUnits::pct);

  }
}

void goSquare2(int N, double a, double speed, double kp) { // clockwise
  for (int i = 0; i < N; ++i) {
      goStraight(a, vex::directionType::fwd, 0, speed, kp);
      makeTurn(90, true, 15, 0.05, 0.5);

      goStraight(a, vex::directionType::fwd, 90, speed, kp);
      makeTurn(180, true, 15, 0.05, 0.5);
 
      goStraight(a, vex::directionType::fwd, 180, speed, kp);
      makeTurn(270, true, 15, 0.05, 0.5);

      goStraight(a, vex::directionType::fwd, 270, speed, kp);
      makeTurn(0, true, 15, 0.05, 0.5);

  }
}

void goSquare3(int N, double a, double speed, double kp) { // counterclockwise
  for (int i = 0; i < N; ++i) {
      goStraight(a, vex::directionType::fwd, 0, speed, kp);
      makeTurn(270, false, 15, 0.05, 0.5);

      goStraight(a, vex::directionType::fwd, 270, speed, kp);
      makeTurn(180, false, 15, 0.05, 0.5);
 
      goStraight(a, vex::directionType::fwd, 180, speed, kp);
      makeTurn(90, false, 15, 0.05, 0.5);

      goStraight(a, vex::directionType::fwd, 90, speed, kp);
      makeTurn(0, false, 15, 0.05, 0.5);

  }
}

void autonomous_test( void ) {
  makeTurn(90, true);
  goStraight(10, vex::directionType::fwd, 90, 50);
}

void autonomous( void ) {

  // calibrate
  inertialSensor.calibrate();
  vex::task::sleep(2000); 

  // set up variables
  double pushSpeed = 80;
  double turnSpeed = 50;
  double tileSize = 23.5;


  goStraight(3.25 * tileSize, vex::directionType::fwd, 0, pushSpeed);
 
  // coming back from alliance mogo
  goStraight(2 * tileSize, vex::directionType::rev, 0, pushSpeed);
  makeTurn(270, false, turnSpeed);

  goStraight(1.1 * tileSize, vex::directionType::fwd, 270, pushSpeed);
  makeTurn(0, true, turnSpeed);
  
  // push neutral mobile goal 1
  goStraight(2 * tileSize, vex::directionType::fwd, 0, pushSpeed);
  vex::task::sleep(500);
 
  // coming back from first neutral mogo
  goStraight(2 * tileSize, vex::directionType::rev, 0, pushSpeed);
  makeTurn(270, false);
  goStraight(1.5 * tileSize, vex::directionType::fwd, 270, pushSpeed);
  makeTurn(0, true, turnSpeed);

  // push neutral mobile goal 2
  goStraight(2 * tileSize, vex::directionType::fwd, 0, pushSpeed);
  makeTurn(0, true, turnSpeed);
  
  // coming back from second neutral mogo
  goStraight(2 * tileSize, vex::directionType::rev, 0, pushSpeed);
  makeTurn(270, false, turnSpeed);
  goStraight(1.5 * tileSize, vex::directionType::fwd, 270, pushSpeed);
  makeTurn(0, true, turnSpeed);

  // pushing third neutral mogo
  goStraight(3.2 * tileSize, vex::directionType::fwd, 0, pushSpeed);

  // pushing blue mogo over to red zone
  goStraight(0.3 * tileSize, vex::directionType::rev, 0, pushSpeed);
  makeTurn(90, true, turnSpeed);
  backintake.spinFor(vex::directionType::fwd, 3.5, vex::rotationUnits::rev);
  goStraight(1.2 * tileSize, vex::directionType::rev, 90, pushSpeed);
  backintake.spinFor(vex::directionType::rev, 1, vex::rotationUnits::rev);
  goStraight(3, vex::directionType::fwd, 90, pushSpeed);
  backintake.spinFor(vex::directionType::rev, 0.5, vex::rotationUnits::rev);
  goStraight(0.5 * tileSize, vex::directionType::fwd, 90, pushSpeed);
  makeTurn(0, false);
  goStraight(3 * tileSize, vex::directionType::rev, 0, pushSpeed);
  vex::task::sleep(500);
}


void usercontrol( void ) {
  double liftSpeed = 70;
  lift.setVelocity(liftSpeed, vex::percentUnits::pct);
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
      backintake.setVelocity(50, vex::percentUnits::pct);
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
      frontintake.setVelocity(50, vex::percentUnits::pct);
      frontintake.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonB.pressing()) {
      frontintake.setVelocity(50, vex::percentUnits::pct);
      frontintake.spin(vex::directionType::rev);
    }
    else {
      frontintake.stop(vex::brakeType::hold);
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