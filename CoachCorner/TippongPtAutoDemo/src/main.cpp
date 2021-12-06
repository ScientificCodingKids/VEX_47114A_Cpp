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
}; // class Drivebase

DriveBase::DriveBase(vex::motor_group& _leftdrive, vex::motor_group& _rightdrive, vex::inertial& inertialSensor):
    _leftdrive(leftdrive), _rightdrive(rightdrive),
    _inertialSensor(inertialSensor) {
}

void DriveBase::resetDriveTrainRotation() {
  this->_leftdrive.resetRotation();

  this->_rightdrive.resetPosition();

  this->_inertialSensor.calibrate();
}

void DriveBase::stopDriveTrain() {
  this->_leftdrive.stop();
  this->_rightdrive.stop();
}

void DriveBase::goStraight(double dist, vex::directionType dt, double tgtHeading, double speed, double kp)
{
  double distToGo = dist;

  while (distToGo < dist) {
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
    this->_leftdrive.setVelocity(speed * ( 1 + kp * headingError), vex::percentUnits::pct);
    this->_rightdrive.setVelocity(speed * (1 - kp * headingError), vex::velocityUnits::pct);

    this->_leftdrive.spin(dt);
    this->_rightdrive.spin(dt);

    vex::task::sleep(50);
    
    distToGo = dist - this->_leftdrive.rotation(vex::rotationUnits::deg) / 360 * (4.0 * 3.14159265); // omni wheel is 4 in diameter
  }
}

DriveBase db(leftdrive, rightdrive, inertial_sensor);


void autonomous( void ) {
  inertial_sensor.calibrate();
  vex::task::sleep(2000); 

  double push_speed = 50;
  lift.rotateFor(vex::directionType::rev, 50, vex::rotationUnits::deg);

  // push near red mobile goal to opposite zone
  db.goStraight(3.2 * 24, vex::directionType::fwd, 0, push_speed);
  db.goStraight(1.8 * 24, vex::directionType::rev, 0, push_speed);

  turnToHeadingWithSleep(sdrive, 270, vex::rotationUnits::deg, push_speed, vex::velocityUnits::pct);

  db.goStraight(22.0, vex::directionType::fwd, 270.0, push_speed);
    
  turnToHeadingWithSleep(sdrive, 0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);

  
  // push neutral mobile goal 1
  db.goStraight(2.0 * 24, vex::directionType::fwd, 0, push_speed);
  db.goStraight(2.0 * 24, vex::directionType::rev, 0, push_speed);
  
  // move to next goal
  turnToHeadingWithSleep(sdrive, 270, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);

  db.goStraight(1.5 * 24, vex::directionType::fwd, 270.0, push_speed);
  sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);

  // push middle neutral mobile goal
  db.goStraight(1.5 * 24, vex::directionType::fwd, 270.0, push_speed);
  db.goStraight(1.5 * 24, vex::directionType::rev, 270.0, push_speed);

  // move to next goal
  turnToHeadingWithSleep(sdrive, 270, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);

  db.goStraight(1.5 * 24, vex::directionType::fwd, 270.0, push_speed);
  sdrive.turnToHeading(0, vex::rotationUnits::deg, 30, vex::velocityUnits::pct);
  
  // push neutral mobile goal 3
  db.goStraight(2.0 * 24, vex::directionType::fwd, 0, push_speed);
  db.goStraight(2.0 * 24, vex::directionType::rev, 0, push_speed);

  // back off and climb up  
  db.goStraight(1.5 * 24, vex::directionType::rev, 0, push_speed);

  turnToHeadingWithSleep(sdrive, 90, vex::rotationUnits::deg, 30, vex::velocityUnits::pct); // face the platform
  lift.rotateFor(vex::directionType::fwd, 150, vex::rotationUnits::deg); // lift up (why need this?)
  sdrive.driveFor(vex::directionType::fwd, 1*24, vex::distanceUnits::in, 70, vex::velocityUnits::pct); // move fast to climb
  sdrive.driveFor(vex::directionType::fwd, 0.5*24, vex::distanceUnits::in, 20, vex::velocityUnits::pct);  // slow down to stay balanced


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