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
#include <cassert>

using namespace vex;
using namespace std;

competition Competition;


void pre_auton( void ) {
}

double rotation2distance(double deg, double gearRatio = 1, double wheelDiameter = 4) {
  // returns distance in inches
  double distance = (gearRatio * deg * wheelDiameter * M_PI) / 360;
  return distance;
}

void goStraight(double dist, vex::directionType dt, double tgtHeading, double originalSpeed, double kp = 0.02, vex::brakeType bt = brake) {
  leftdrive.resetRotation();
  rightdrive.resetRotation();

  double distToGo = dist;
  double distTravelled = dist - distToGo;
  double finalSpeed = 20;
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

    distToGo = dist - rotation2distance(fabs(leftdrive.rotation(vex::rotationUnits::deg)));
    distTravelled = dist - distToGo;
  }
  cout << "finalSpeed = " << speed << endl;
  Brain.Screen.print("finalSpeed = %f ", speed);
  leftdrive.stop(bt);
  rightdrive.stop(bt);
}


double makeTurn(double tgtHeading, bool turnClockwise, double origSpeed=15, double tol=0.1, double adaptiveInterval=15, vex::brakeType bt = brake, bool makeStop = true)
{
  leftdrive.resetRotation();
  rightdrive.resetRotation();

  double degreeToGo = tgtHeading - inertialSensor.heading();

  if (degreeToGo < 0) {
    degreeToGo = degreeToGo + 360.0;
  }

  while (degreeToGo > tol) {
    // all sensor values should be read only once in each while loop iteration
    double ch = inertialSensor.heading();

    //1. compute cw, ccw degreeToGo
    double CWDegreeToGo = tgtHeading - ch;
    double CCWDegreeToGo = 360 - CWDegreeToGo;
    
    if (CWDegreeToGo < 0) {
      CWDegreeToGo = CWDegreeToGo + 360;
      CCWDegreeToGo = 360 - CWDegreeToGo;
    }

    assert( CWDegreeToGo >= 0 && CWDegreeToGo < 360);
    assert( CCWDegreeToGo >=0 && CCWDegreeToGo < 360);

    bool isClose = min(CWDegreeToGo, CCWDegreeToGo) < adaptiveInterval;

    bool currentTurnClockwise = turnClockwise;

    if (isClose) {
      currentTurnClockwise = CWDegreeToGo < CCWDegreeToGo;
    }

    // double degreeToGo = if currentheading then CWDegreeToGo else CCWDegreeToGo
    degreeToGo = currentTurnClockwise ? CWDegreeToGo : CCWDegreeToGo;

    double currentSpeed = origSpeed;

    if (isClose) {
      currentSpeed = origSpeed * (1- (adaptiveInterval - degreeToGo)/adaptiveInterval);

      //if (distToGo < adaptiveInterval) speed = originalSpeed * (1 - (adaptiveInterval - distToGo) / adaptiveInterval);

    }

    // 3. set motor speed and direction
    Brain.Screen.print("%f, %d \n", currentSpeed, isClose);
    cout << inertialSensor.heading() << ": [ " << CWDegreeToGo << ", " << CCWDegreeToGo << "]" << degreeToGo << ", " << currentSpeed << "; " << isClose << endl;  // print to terminal

   
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
  } // while loop
  
  if (makeStop) {
    leftdrive.stop(bt);
    rightdrive.stop(bt);
  }

  Brain.Screen.print("done");
  return inertialSensor.heading();

} // maketurn


void autonomous( void ) {
  double currentheading = 0;
  double tileSize = 23.5;
  double driveSpeed = 70;
  double turnSpeed = 40;
  lift.setVelocity(60, vex::percentUnits::pct);

  // calibrate
  inertialSensor.calibrate();
  vex::task::sleep(1500);

  // scoop alliance off of platform
  backintake.spinFor(vex::directionType::fwd, 3.5, vex::rotationUnits::rev, 80, vex::velocityUnits::pct);
  goStraight(0.8 * tileSize, vex::directionType::rev, currentheading, driveSpeed);
  backintake.spinFor(vex::directionType::rev, 2, vex::rotationUnits::rev, 75, vex::velocityUnits::pct, false);
  vex::task::sleep(500);
  goStraight(0.8, vex::directionType::fwd, currentheading, driveSpeed);

  // get first neutral
  currentheading = makeTurn(93, true, turnSpeed);
  goStraight(2.3 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);
  frontintake.spinFor(vex::directionType::rev, 120, vex::rotationUnits::deg, 80, vex::velocityUnits::pct, false);
  vex::task::sleep(500);
  lift.spinFor(vex::directionType::fwd, 60, vex::rotationUnits::deg);
  frontintake.spin(vex::directionType::rev, 10, vex::percentUnits::pct);
  goStraight(0.9 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);

  // drop alliance mogo
  currentheading = makeTurn(180, true, turnSpeed);
  lift.setVelocity(40, vex::percentUnits::pct);
  lift.spinFor(vex::directionType::fwd, 600, vex::rotationUnits::deg, false);
  backintake.spinFor(vex::directionType::fwd, 2.2, vex::rotationUnits::rev, 80, vex::velocityUnits::pct);
  goStraight(1 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);
  goStraight(0.4 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);
  backintake.spinFor(vex::directionType::rev, 3.5, vex::rotationUnits::rev, 80, vex::velocityUnits::pct, false);
  vex::task::sleep(500);
  
  // stack yellow
  currentheading = makeTurn(90, false, turnSpeed);
  goStraight(1.2 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);
  lift.spinFor(vex::directionType::rev, 200, vex::rotationUnits::deg, 80, vex::velocityUnits::pct);
  frontintake.stop();
  frontintake.spinFor(vex::directionType::fwd, 120, vex::rotationUnits::deg, 80, vex::velocityUnits::pct);
  lift.spinFor(vex::directionType::fwd, 200, vex::rotationUnits::deg);
  goStraight(1.2 * tileSize, vex::directionType::rev, currentheading, driveSpeed);
  lift.spinFor(vex::directionType::rev, 660, vex::rotationUnits::deg, false);
  vex::task::sleep(500);

  // grab alliance mogo
  currentheading = makeTurn(0, false, turnSpeed);
  goStraight(2 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);
  frontintake.spinFor(vex::directionType::rev, 100, vex::rotationUnits::deg, 80, vex::velocityUnits::pct, false);
  vex::task::sleep(500);
  frontintake.spin(vex::directionType::rev, 10, vex::percentUnits::pct);
  lift.spinFor(vex::directionType::fwd, 60, vex::rotationUnits::deg);
  goStraight(2.3 * tileSize, vex::directionType::rev, currentheading, driveSpeed);

  // stack alliance mogo
  lift.spinFor(vex::directionType::fwd, 600, vex::rotationUnits::deg, false);
  currentheading = makeTurn(90, true, turnSpeed);
  vex::task::sleep(500);
  goStraight(1.2 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);
  lift.spinFor(vex::directionType::rev, 180, vex::rotationUnits::deg);
  frontintake.stop();
  frontintake.spinFor(vex::directionType::fwd, 120, vex::rotationUnits::deg, 80, vex::velocityUnits::pct);
  lift.spinFor(vex::directionType::fwd, 150, vex::rotationUnits::deg, false);
  goStraight(1.2 * tileSize, vex::directionType::rev, currentheading, driveSpeed);
  lift.spinFor(vex::directionType::rev, 660, vex::rotationUnits::deg, false);

  // scoop alliance from under platform
  currentheading = makeTurn(180, true, turnSpeed);
  backintake.spinFor(vex::directionType::fwd, 3.5, vex::rotationUnits::rev, 75, vex::velocityUnits::pct, false);
  goStraight(2.1 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);
  currentheading = makeTurn(240, true, turnSpeed);
  goStraight(1.65 * tileSize, vex::directionType::rev, currentheading, driveSpeed);
  backintake.spinFor(vex::directionType::rev, 1, vex::rotationUnits::rev, 75, vex::velocityUnits::pct, false);
  vex::task::sleep(500);
  goStraight(1.2 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);
  backintake.spinFor(vex::directionType::rev, 1, vex::rotationUnits::rev, 75, vex::velocityUnits::pct, false);

  // grab next
  currentheading = makeTurn(305, true, turnSpeed);
  goStraight(1.3 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);
  frontintake.spinFor(vex::directionType::rev, 120, vex::rotationUnits::deg, 80, vex::velocityUnits::pct, false);
  vex::task::sleep(500);
  lift.spinFor(vex::directionType::fwd, 660, vex::rotationUnits::deg, false);
  frontintake.spin(vex::directionType::rev, 10, vex::percentUnits::pct);
  goStraight(2.3 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);
  lift.spinFor(vex::directionType::rev, 180, vex::rotationUnits::deg);
  frontintake.stop();
  frontintake.spinFor(vex::directionType::fwd, 120, vex::rotationUnits::deg, 80, vex::velocityUnits::pct);
  lift.spinFor(vex::directionType::fwd, 150, vex::rotationUnits::deg);
  goStraight(0.5 * tileSize, vex::directionType::rev, currentheading, driveSpeed);
  currentheading = makeTurn(250, false, turnSpeed);
  goStraight(2 * tileSize, vex::directionType::rev, currentheading, driveSpeed);
  goStraight(2 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);
}


void usercontrol( void ) {
  double liftSpeed = 70;
  bool pressIn = false;
  bool deployFork = false;
  bool climbPlatform = false;
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


    double leftMotorSpeed = rc.Axis3.position(vex::percentUnits::pct) * 0.85;
    double rightMotorSpeed = rc.Axis2.position(vex::percentUnits::pct) * 0.85;

    if (((rightMotorSpeed-2) < leftMotorSpeed) && (leftMotorSpeed < (rightMotorSpeed+2))) {
      rightMotorSpeed = leftMotorSpeed;
    }

    if (fabs(leftMotorSpeed) > 5.0) {
      backleftdrive.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      frontleftdrive.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      backleftdrive.spin(fwd);
      frontleftdrive.spin(fwd);
    }
    else if (rc.ButtonUp.pressing()) {
      leftdrive.spin(vex::directionType::fwd, 80, vex::percentUnits::pct);
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
    else if (rc.ButtonUp.pressing()) {
      rightdrive.spin(vex::directionType::fwd, 80, vex::percentUnits::pct);
    }
    else {
      if (rc.ButtonLeft.pressing()) {
        climbPlatform = true;
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
    if (deployFork == true) {
      backintake.spinFor(vex::directionType::fwd, 3.6, vex::rotationUnits::rev, 80, vex::velocityUnits::pct);
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