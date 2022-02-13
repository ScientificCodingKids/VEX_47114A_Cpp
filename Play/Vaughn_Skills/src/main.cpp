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
}

void turnToHeadingWithSleep(vex::smartdrive& sd, double tgt, vex::rotationUnits ru, double speed, vex::velocityUnits vu)
{
  double nap = 300;

  vex::task::sleep(nap);
  sd.turnToHeading(tgt, vex::rotationUnits::deg, speed, vu);
  vex::task::sleep(nap);
}

void goStraight(double dist, vex::directionType dt, double tgtHeading, double speed, double kp = 0.01) {
  leftdrive.resetRotation();
  rightdrive.resetRotation();

  double distToGo = dist;

  while (distToGo > 0) {
    double headingError = inertialSensor.heading() - tgtHeading;
    if (headingError < -270) headingError = headingError + 360;
    if (headingError > 270) headingError = headingError - 360;

    if (headingError < -15) headingError = -15;
    if (headingError > 15) headingError = 15;

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

void makeTurn(double tgtHeading, bool turnClockwise, double speed=15, double kp=0.05, double kd=0, double tol=0.5)
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

    bool isClose = false;

    if (headingError > 15) {
      headingError = 15;
      currentSpeed = speed;
      isClose = true;
    }

    if (headingError < -15) {
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

    if (currentSpeed < 2) {
      currentSpeed = 2;
    }

    // 3. set motor speed and direction
    Brain.Screen.print("%f, %d \n", currentSpeed, isClose);

   
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

  vex::task::sleep(2000);
 
}


void autonomous( void ) {

  // calibrate
  inertialSensor.calibrate();
  vex::task::sleep(1000); 

  // set up variables
  double pushSpeed = 60;
  double tileSize = 23.5;

  // push first alliance mogo
  //leftintake.spinFor(vex::directionType::rev, 0.3, vex::rotationUnits::rev);
  //leftintake.spinFor(vex::directionType::fwd, 0.1, vex::rotationUnits::rev);
  //lift.rotateFor(vex::directionType::rev, 80, vex::rotationUnits::deg);
  goStraight(3.25 * tileSize, vex::directionType::fwd, 0, pushSpeed);
 
  // coming back from alliance mogo
  goStraight(1.75 * tileSize, vex::directionType::rev, 0, pushSpeed);
  makeTurn(270, false);
  goStraight(0.8 * tileSize, vex::directionType::fwd, 270, pushSpeed);
  makeTurn(0, true);
  
  // push neutral mobile goal 1
  goStraight(2 * tileSize, vex::directionType::fwd, 0, pushSpeed);
  vex::task::sleep(500);
 
  // coming back from first neutral mogo
  goStraight(2 * tileSize, vex::directionType::rev, 0, pushSpeed);
  makeTurn(270, false);
  goStraight(1.15 * tileSize, vex::directionType::fwd, 270, pushSpeed);
  makeTurn(0, true);

  // push neutral mobile goal 2
  goStraight(2 * tileSize, vex::directionType::fwd, 0, pushSpeed);
  makeTurn(0, true);
  
  // coming back from second neutral mogo
  goStraight(2 * tileSize, vex::directionType::rev, 0, pushSpeed);
  makeTurn(270, false);
  goStraight(1.2 * tileSize, vex::directionType::fwd, 270, pushSpeed);
  makeTurn(0, true);

  // pushing third neutral mogo
  goStraight(3.1 * tileSize, vex::directionType::fwd, 0, pushSpeed);
  vex::task::sleep(100);
  goStraight(0.15 * tileSize, vex::directionType::rev, 0, pushSpeed);
  
  // grab the blue mogo and bring it back to red zone
  makeTurn(270, false);
  goStraight(0.55 * tileSize + 2, vex::directionType::fwd, 270, 30, false);
  vex::task::sleep(1000);

  leftintake.rotateFor(vex::directionType::rev, 0.2, vex::rotationUnits::rev, false);
  vex::task::sleep(500);
  lift.rotateFor(vex::directionType::fwd, 10, vex::rotationUnits::deg);
  goStraight(10, vex::directionType::rev, 270, 70);
  lift.rotateFor(vex::directionType::fwd, 40, vex::rotationUnits::deg);
  makeTurn(0, true);
  goStraight(3.3 * tileSize, vex::directionType::rev, 0, pushSpeed);
 
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