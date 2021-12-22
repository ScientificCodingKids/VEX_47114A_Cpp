/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\King                                             */
/*    Created:      Sun Dec 05 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/


// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <iostream>

using namespace vex;
using namespace std;

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

class DriveBase {
private:
    vex::motor_group& _leftdrive;
    vex::motor_group& _rightdrive;

    vex::inertial& _inertialSensor; 
public:
    DriveBase(vex::motor_group& leftdrive, vex::motor_group& rightdrive, vex::inertial& inertialSensor);
    void resetDriveTrainRotation();
    void stopDriveTrain();

    //void makeTurn(double baseSpeed, double minStartSpeed, double minEndSpeed, bool turnLeft);
    void goStraight(double dist, vex::directionType dt, double tgtHeading, double speed, double kp = 0.01); // P(ID) on heading!
    // void goStraightWithGyro(double rotationsToGo, double baseSpeed, double minStartSpeed, double minEndSpeed, double kp);

    void makeTurn(double tgtHeading, bool turnLeft, double speed, double kp = 0.01, double kd = 0.001, double tol=0.5); // P(I)D on heading
}; // class Drivebase

DriveBase::DriveBase(vex::motor_group& _leftdrive, vex::motor_group& _rightdrive, vex::inertial& inertialSensor):
    _leftdrive(leftdrive), _rightdrive(rightdrive),
    _inertialSensor(inertialSensor) {
}

void DriveBase::resetDriveTrainRotation() {
  this->_leftdrive.resetRotation();

  this->_rightdrive.resetRotation();

}

void DriveBase::stopDriveTrain() {
  this->_leftdrive.stop();
  this->_rightdrive.stop();
}

void balanceOnPlatform(vex::motor_group& leftdrive, vex::motor_group& rightdrive, double baseSpeed = 50, double kp = 0.1)
{
  // assume inertial sensor already calibrated on flat surface before calling this function
  leftdrive.resetRotation();
  rightdrive.resetRotation();

  cout << "pitch=" << inertial_sensor.pitch() << endl;
  Brain.Screen.print("pitch=%f", inertial_sensor.pitch());

  task::sleep(2000);

  double pitchErr = inertial_sensor.pitch();

  while (fabs(pitchErr) > 2.0) {
      // if (pitchErr < -10) pitchErr = -10;
      // if (pitchErr > 10) pitchErr = 10;

      double speed = baseSpeed * kp * pitchErr;

      if (speed > 50) speed = 50;

      cout << "pitchErr=" << pitchErr << ", speed=" << speed << endl;
      Brain.Screen.print("pitch=%f, speed=%f", inertial_sensor.pitch(), speed);

      leftdrive.setVelocity(speed, vex::percentUnits::pct);
      rightdrive.setVelocity(speed, vex::percentUnits::pct);

      if (pitchErr > 0) {
        leftdrive.spin(vex::directionType::rev);
        rightdrive.spin(vex::directionType::rev);
      }
      else {
        leftdrive.spin(vex::directionType::fwd);
        rightdrive.spin(vex::directionType::fwd);
      }

      vex::task::sleep(10);

      pitchErr = inertial_sensor.pitch();
  }
  leftdrive.stop();
  rightdrive.stop();
}

void goStraight(vex::motor_group& leftdrive, vex::motor_group& rightdrive, double dist, vex::directionType dt, double tgtHeading, double speed, double kp = 0.01) {
  leftdrive.resetRotation();
  rightdrive.resetRotation();

  double distToGo = dist;

  while (distToGo > 0) {
    double headingError = inertial_sensor.heading() - tgtHeading;
    if (headingError < -270) headingError = headingError + 360;
    if (headingError > 270) headingError = headingError - 360;

    if (headingError < -5) headingError = -5;
    if (headingError > 5) headingError = 5;

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

void DriveBase::goStraight(double dist, vex::directionType dt, double tgtHeading, double speed, double kp)
{
  this->resetDriveTrainRotation();
  double distToGo = dist;

  while (distToGo > 0) {
    double headingError = tgtHeading - this->_inertialSensor.heading();
    if (headingError < -270) headingError = headingError + 360;
    if (headingError > 270) headingError = headingError - 360;
    // now -90 < headingError < 90
    
    // avoid noisy reading, capping the error at -5, +5
    if (headingError < -5) headingError = -5;
    if (headingError > 5) headingError = 5;

    // positive headingError means current heading is "to the left" of target heading, robot need make a right turn adj
    // this means we should increase speed of left motors and decrease speed of right motors
    // but, do not make speed adjustment too big or the robot will zig-zag
    if (dt == vex::directionType::fwd) {
    this->_leftdrive.setVelocity(speed * ( 1 + kp * headingError), vex::percentUnits::pct);
    this->_rightdrive.setVelocity(speed * (1 - kp * headingError), vex::velocityUnits::pct);
    } else {
      this->_leftdrive.setVelocity(speed * ( 1 - kp * headingError), vex::percentUnits::pct);
      this->_rightdrive.setVelocity(speed * (1 + kp * headingError), vex::velocityUnits::pct);
    }
    this->_leftdrive.spin(dt);
    this->_rightdrive.spin(dt);

    vex::task::sleep(10);
    
    distToGo = dist - fabs(this->_leftdrive.rotation(vex::rotationUnits::deg)) / 360 * (4.0 * 3.14159265); // omni wheel is 4 in diameter
  }

  this->stopDriveTrain();
}


void DriveBase::makeTurn(double tgtHeading, bool turnClockwise, double speed, double kp, double kd, double tol)
{
  this->resetDriveTrainRotation();

  double degreeToGo = tgtHeading - this->_inertialSensor.heading();
  
  if (turnClockwise) { // 0 -> 90; 0 -> 270, 270 -> 90
    if (degreeToGo < 0) {
      degreeToGo = degreeToGo + 360.0;
    }
  }
  else  {  //counter clockwise 0 -> 90; 0 -> 270, 270 -> 90
    // FIX ME
    if (degreeToGo < 0) {
      degreeToGo = degreeToGo + 360.0;
    }
  }

  while (fabs(degreeToGo) > tol) {
    double headingError = degreeToGo;

    if (headingError > 15) {
      headingError = 15;
    }

    if (headingError < -15) {
      headingError = -15;
    }

    double currentSpeed = speed * (1 - kp * headingError); // when close to target heading, the speed should be low (but not 0)

    this->_leftdrive.setVelocity(currentSpeed, vex::percentUnits::pct);
    this->_rightdrive.setVelocity(currentSpeed, vex::velocityUnits::pct);
    
    if (turnClockwise) {
      this->_leftdrive.spin(vex::directionType::fwd);
      this->_rightdrive.spin(vex::directionType::rev);
    }
    else {
      // FIX ME
    }

    vex::task::sleep(10);
    
    degreeToGo = tgtHeading - this->_inertialSensor.heading(); 
    //FIX ME

    // warning: when current heading is very close to target heading, the turn direction is always the "quickest" turn direction to hit target -- not necessary to match the input turnClockwise
  }

  this->stopDriveTrain();
}


DriveBase db(leftdrive, rightdrive, inertial_sensor);


void autonomous( void ) {
  inertial_sensor.calibrate();
  vex::task::sleep(2000); 

  balanceOnPlatform(leftdrive, rightdrive, 50, 0.1);
}

void autonomous1( void ) {
  inertial_sensor.calibrate();
  vex::task::sleep(2000); 

  double push_speed = 50;
  double tileSize = 23.5;

  lift.rotateFor(vex::directionType::rev, 50, vex::rotationUnits::deg);

  // push near red mobile goal to opposite zone
  db.goStraight(3.2 * tileSize, vex::directionType::fwd, 0, push_speed);
  db.goStraight(1.6 * tileSize, vex::directionType::rev, 0, push_speed);

  turnToHeadingWithSleep(sdrive, 270, vex::rotationUnits::deg, push_speed, vex::velocityUnits::pct);

  vex::task::sleep(5000); // just to see if 270 is correct
  db.goStraight(22.0, vex::directionType::fwd, 270.0, push_speed/2);
  
  turnToHeadingWithSleep(sdrive, 0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);

  
  // push neutral mobile goal 1
  db.goStraight(2.0 * tileSize, vex::directionType::fwd, 0, push_speed);
  vex::task::sleep(500);
  db.goStraight(2.0 * tileSize, vex::directionType::rev, 0, push_speed);
  
  // move to next goal
  turnToHeadingWithSleep(sdrive, 270, vex::rotationUnits::deg, push_speed, vex::velocityUnits::pct);
  vex::task::sleep(5000); // just to see if 270 is correct

  db.goStraight(1.4 * tileSize, vex::directionType::fwd, 270.0, push_speed/2);
  turnToHeadingWithSleep(sdrive, 0, vex::rotationUnits::deg, push_speed, vex::velocityUnits::pct);

  // push middle neutral mobile goal
  db.goStraight(1.6 * tileSize, vex::directionType::fwd, 0.0, push_speed);
  vex::task::sleep(500);
  db.goStraight(1.6 * tileSize, vex::directionType::rev, 0.0, push_speed);

  // move to next goal
  turnToHeadingWithSleep(sdrive, 270, vex::rotationUnits::deg, push_speed, vex::velocityUnits::pct);

  db.goStraight(1.5 * tileSize, vex::directionType::fwd, 270.0, push_speed);
    turnToHeadingWithSleep(sdrive, 0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);

  
  // push mobile goal 3

  db.goStraight(1.6 * tileSize, vex::directionType::fwd, 0, push_speed);
  vex::task::sleep(500);
  db.goStraight(1.6 * tileSize, vex::directionType::rev, 0, push_speed);

  // back off and climb up  

  turnToHeadingWithSleep(sdrive, 45, vex::rotationUnits::deg, 30, vex::velocityUnits::pct); // face the platform
  db.goStraight(1.6 * tileSize, vex::directionType::rev, 0, push_speed);
  turnToHeadingWithSleep(sdrive, 90, vex::rotationUnits::deg, 30, vex::velocityUnits::pct); // face the platform

  lift.rotateFor(vex::directionType::fwd, 150, vex::rotationUnits::deg); // lift up (why need this?)

  db.goStraight(1. * tileSize, vex::directionType::fwd, 90.0, 70);  // move fast to climb up to the platform
  db.goStraight(0.5 * tileSize, vex::directionType::fwd, 90.0, 20);  // slow down to stay balanced


}

void usercontrol( void ) {
  // int spin = 0;

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