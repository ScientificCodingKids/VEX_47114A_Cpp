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

using namespace vex;
using namespace std;

competition Competition;


void pre_auton( void ) {
}

class Coord {
// the class for 2D (x,y) coordinate system
public:
  double x, y;

  Coord(double x0, double y0): x(x0), y(y0) {; }

};  //class Coord

void sample_sin_cos_code() {
  // In C++, sin(), cos() are declared in cmath header
  // input should be in arc unit, not degree
  // M_PI = 3.14159265... the pi value declared in cmath
  cout << sin(30.0 / M_PI);
  cout << cos(45.0 / M_PI);
}

Coord goStraight(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, double kp = 0.02, vex::brakeType bt = brake, Coord srcLoc = Coord(0.0, 0.0)) {
  Coord destLoc = srcLoc;

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

  return destLoc;
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

    if (headingError > 15) {
      headingError = 15;
      currentSpeed = speed;
    }

    else if (headingError < -15) {
      headingError = -15;
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

void autonomous_test_back_lifter( void ) { // tests only scooping mogo with back lifter
  // starting position of back lifter up as far as it can go
  backintake.spinFor(vex::directionType::fwd, 3.7, vex::rotationUnits::rev);
  goStraight(10, vex::directionType::rev, 0, 50);
  backintake.spinFor(vex::directionType::rev, 1.5, vex::rotationUnits::rev);
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
  makeTurn(270, false, turnSpeed);
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
  // push only:
  goStraight(0.1 * tileSize, vex::directionType::fwd, 0, pushSpeed);

  // pushing blue mogo over to red zone
  // goStraight(0.3 * tileSize, vex::directionType::rev, 0, pushSpeed);
  // makeTurn(90, true, turnSpeed);
  // backintake.spinFor(vex::directionType::fwd, 3.5, vex::rotationUnits::rev);
  // goStraight(1.2 * tileSize, vex::directionType::rev, 90, pushSpeed);
  // backintake.spinFor(vex::directionType::rev, 1, vex::rotationUnits::rev);
  // goStraight(3, vex::directionType::fwd, 90, pushSpeed);
  // backintake.spinFor(vex::directionType::rev, 0.5, vex::rotationUnits::rev);
  // goStraight(0.5 * tileSize, vex::directionType::fwd, 90, pushSpeed);
  // makeTurn(0, false);
  // goStraight(3 * tileSize, vex::directionType::rev, 0, pushSpeed);
  // vex::task::sleep(500);

  // different method: push only!
  goStraight(0.3 * tileSize, vex::directionType::rev, 0, pushSpeed);
  backintake.spinFor(vex::directionType::fwd, 3.75, vex::rotationUnits::rev, 75, vex::velocityUnits::pct);
  makeTurn(80, true, turnSpeed);
  goStraight(0.9 * tileSize, vex::directionType::rev, 0, pushSpeed + 10);
  dt.driveFor(vex::directionType::rev, 0.3*tileSize, vex::distanceUnits::in, 70, vex::velocityUnits::pct);
  makeTurn(0, false, turnSpeed);
  goStraight(2.5 * tileSize, vex::directionType::rev, 0, 100);

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