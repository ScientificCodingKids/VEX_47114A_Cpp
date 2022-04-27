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

void deployLift(double speed=80) {
  backintake.spinFor(vex::directionType::fwd, 3.6, vex::rotationUnits::rev, speed, vex::velocityUnits::pct);
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


double makeTurn(double tgtHeading, bool turnClockwise, double speed=15, double kp=0.03, double tol=0.1)
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

    bool currentTurnClockwise = turnClockwise;

    if (headingError > 15) {
      currentSpeed = speed;
    }

    else if (headingError < -15) {
      currentSpeed = speed;
    }
    else {
      isClose = true;
    }

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
  } // while loop
  
 
  leftdrive.stop();
  rightdrive.stop();
  Brain.Screen.print("done");
  return inertialSensor.heading();

} // maketurn


void autonomous( void ) {
  double currentheading = 0;
  double driveSpeed = 80;
  double turnSpeed = 50;
  double tileSize = 23.5;
  backintake.setVelocity(80, vex::percentUnits::pct);

  // calibrate
  inertialSensor.calibrate();
  vex::task::sleep(1500);

  // left side wp, pick up alliance mogo
  deployLift();
  goStraight(0.8 * tileSize, vex::directionType::rev, currentheading, driveSpeed);
  backintake.spinFor(vex::directionType::rev, 2, vex::rotationUnits::rev, 75, vex::velocityUnits::pct, false);
  vex::task::sleep(500);
  goStraight(0.8, vex::directionType::fwd, currentheading, driveSpeed);

  // get first yellow mogo
  currentheading = makeTurn(92, true, turnSpeed);
  goStraight(2 * tileSize, vex::directionType::fwd, currentheading, driveSpeed);
  frontintake.spinFor(vex::directionType::rev, 120, vex::rotationUnits::deg, 80, vex::velocityUnits::pct, false);
  vex::task::sleep(500);
  lift.spinFor(vex::directionType::fwd, 60, vex::rotationUnits::deg);
  frontintake.spin(vex::directionType::rev, 10, vex::percentUnits::pct);
  goStraight(1.5 * tileSize, vex::directionType::rev, currentheading, driveSpeed);
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

    if (fabs(leftMotorSpeed) > 5.0) {
      backleftdrive.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      frontleftdrive.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      backleftdrive.spin(fwd);
      frontleftdrive.spin(fwd);
    }
    
    
    if (fabs(rightMotorSpeed) > 5.0) {
      backrightdrive.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      backrightdrive.spin(fwd);  
      frontrightdrive.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      frontrightdrive.spin(fwd);
    }

    else if (rc.ButtonUp.pressing()) {
      leftdrive.spin(vex::directionType::fwd, 80, vex::percentUnits::pct);
      rightdrive.spin(vex::directionType::fwd, 80, vex::percentUnits::pct);
    }

    else {
      backrightdrive.setBrake(vex::brakeType::coast);
      backleftdrive.setBrake(vex::brakeType::coast);
      frontrightdrive.setBrake(vex::brakeType::coast);
      frontleftdrive.setBrake(vex::brakeType::coast);
      if (rc.ButtonLeft.pressing()) {
        climbPlatform = true;
        backrightdrive.setBrake(vex::brakeType::brake);
        backleftdrive.setBrake(vex::brakeType::brake);
        frontrightdrive.setBrake(vex::brakeType::brake);
        frontleftdrive.setBrake(vex::brakeType::brake);
      }
      leftdrive.stop();
      rightdrive.stop();
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