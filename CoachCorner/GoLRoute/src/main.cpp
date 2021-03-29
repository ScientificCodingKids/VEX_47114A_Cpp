/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\King                                             */
/*    Created:      Sat Jan 23 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

// test if negative speed will result in opposite direction movement

#include <algorithm>
#include "vex.h"

#include "../../../Utils/nutils.hpp"

using namespace vex;
competition Competition;

// use heading(), data range [0, 360]. so calibrate to 0 means the initial reading can be 365.5 !
auto ss0 = ScrollingScreen<int>(1, 2);
auto ss = ScrollingScreen<int>(3, 10);


double computeDistanceForOneRotation() {
  // return the wheel travel distance (in inch) when motor spins one rotation
  double wheelDiameter = 4.0; // omni "green" wheel
  double wheelToDriveGearRatio = 1.0;
  
  return 3.14159265 * wheelDiameter / wheelToDriveGearRatio;
}


double computeRotationsFromDistance(double x /*inch*/) {
  return x / computeDistanceForOneRotation();
}

void resetDriveTrainRotation() {
  backleftdrive.resetPosition();
  frontleftdrive.resetPosition();

  backrightdrive.resetPosition();
  frontrightdrive.resetPosition();
}

void stopDriveTrain() {
  backleftdrive.stop();
  frontleftdrive.stop();

  backrightdrive.stop();
  frontrightdrive.stop();

  backleftdrive.index();
}


void makeTurn(double baseSpeed, double minStartSpeed, double minEndSpeed, bool turnLeft) {
  // auto rotation using simplied PID (P-only) with help from inertial sensor
  // recommended baseSpeed is 50 (normal), 10 (slow)
  // requirement:
  // 1) inertialSensor takes values clock-wise (WARN: its reading is in range(0, 360), NO negative reading)
  // i.e. it starts at 0, a slight right reads a small positive value, a slight left reads a value slightly below 360
  // 2) leftDriveMotor and rightDriveMotor are both forward spinning
  //
  // PID algorithm in use:
  // the approaching speed is computed from the approaching error (when within 10 degrees)

  // INITIALIZATION
  inertialSensor.calibrate();

  bool is_done = false;

  while (inertialSensor.isCalibrating()) {
    vex::wait(50, timeUnits::msec);
  }

  inertialSensor.resetRotation();

  resetDriveTrainRotation();

  double speed = 0.0;

  ss.print("Initial: %.1f", inertialSensor.rotation());

  // MOVEMENT LOOP
  // while (((inertialSensor.heading() < 95.0) || (inertialSensor.heading() > 265)) && ~is_done) {

  while (!is_done) {
    // set baseline speed
    speed = baseSpeed;

    // for control purpose, we focus on r, not the raw reading from inertial sensor
    // r: start from 0; orientation free 
    //  i.e. 
    //  r= 90 means 
    //  -- it made 90 deg LEFT if turnLeft=True; 
    //  -- it made 90 deg RIGHT if turnLeft=False.

    double r = turnLeft ? (360 - inertialSensor.rotation()): inertialSensor.rotation(); 

    if (r > 180) {
      r = r - 360;
    }

    // compute "effective" speed depending on movement phases
    // 1) start (ramp-up) phase
    if (r < 10.0) {
      speed = std::min(baseSpeed, std::max(minStartSpeed, baseSpeed * (r / 10.0)));
    }

    // 2) normal phase

    // 3) end (approaching) phase
    directionType diLeftWheels = turnLeft ? directionType::rev : directionType::fwd;
    directionType diRightWheels = turnLeft ? directionType::fwd : directionType::rev;

    if (r > 90.0) {
      speed = std::min(5.0, baseSpeed * (r-90) / 10.0);
      // flip
      diLeftWheels = (diLeftWheels == directionType::fwd) ? directionType::rev : directionType::fwd;
      diRightWheels = (diRightWheels == directionType::fwd) ? directionType::rev : directionType::fwd;
    }
    else if (r > 80.0) {
      speed = std::max(minEndSpeed, baseSpeed * (1.0 - (r - 80.0)/ 10.0));
    }
    if (std::abs(90-r) < 0.2) {
      is_done = true;
      speed = 0.0;
    }
    // drive the motors using equal speeds but opposite direction to turn
    backleftdrive.setVelocity(speed, vex::percentUnits::pct);
    frontleftdrive.setVelocity(speed, vex::percentUnits::pct);

    backrightdrive.setVelocity(speed, vex::percentUnits::pct);
    frontrightdrive.setVelocity(speed, vex::percentUnits::pct);

    backleftdrive.spin(diLeftWheels);
    frontleftdrive.spin(diLeftWheels);
    
    backrightdrive.spin(diRightWheels);
    frontrightdrive.spin(diRightWheels);

    ss.print("Rotate %.1f , %.1f, %s", r, speed, is_done? "true": "false");
    vex::task::sleep(10);
  } // while

  // FINALE
  stopDriveTrain();

  ss.print("makeTurn() ENDS");
}


void goStraight(double rotationsToGo, double baseSpeed, double minStartSpeed, double minEndSpeed, double kp, bool goReverse=false, bool useGyro=false) {
  // auto rotation using simplied PID (P-only) without help from inertial sensor
  // recommended baseSpeed is 30 (normal), 10 (slow)
  // kp = 0 (no P-adj), kp = 0.005 (small), kp = 0.1 (extremely large, ONLY for demo purpose)
  // requirement:
  // 1) inertialSensor takes values clock-wise (WARN: its reading is in range(0, 360), NO negative reading)
  // i.e. it starts at 0, a slight right reads a small positive value, a slight left reads a value slightly below 360
  // 2) leftDriveMotor and rightDriveMotor are both forward spinning
  //
  // PID algorithm in use:
  // the deviation in forward direction is computed as the diffence of rotations between left and right motors
  // P-only algorithm modifies the speed on both motors to suppress above deviation so that robot goes straight forward

  double GyroAdjust = 0;

  // CALIBRATE GYRO

  inertialSensor.calibrate();

  while (inertialSensor.isCalibrating()) {
    vex::wait(50, timeUnits::msec);
  }

  inertialSensor.resetRotation();

  // INITIALIZATION
  resetDriveTrainRotation();

  // MOVEMENT LOOP
  while (std::abs(backleftdrive.rotation(vex::rotationUnits::deg)) < rotationsToGo * 360) {
    // set baseline speed
    double speed = baseSpeed;
    double leftRot = std::abs(backleftdrive.rotation(vex::rotationUnits::deg));
    double rightRot = std::abs(backrightdrive.rotation(vex::rotationUnits::deg));

    // for control purpose, we focus on err
    double err = leftRot - rightRot;
    // if (err > 180.0) {
    //   err = 360.0 - err;
    // }
    // compute "effective" speed depending on movement phases
    // start (ramp-up) phase
    if (leftRot < 360) {
      speed = std::max(minStartSpeed, baseSpeed * leftRot / 360.0);
    }
    else if (rotationsToGo * 360 - leftRot < 360) { // end (approaching) ramp down phase
      speed = std::max(minEndSpeed, baseSpeed * (rotationsToGo * 360 - leftRot)/360.0);
    }
    else { // normal  phase
      speed = baseSpeed;
    }

    // drive the motors using equal+adj speeds in same direction
    backleftdrive.setVelocity(speed - speed * err * kp, vex::percentUnits::pct);
    frontleftdrive.setVelocity(speed - speed * err * kp, vex::percentUnits::pct);

    backrightdrive.setVelocity(speed + speed * err * kp, vex::percentUnits::pct);
    frontrightdrive.setVelocity(speed + speed * err * kp, vex::percentUnits::pct);

    directionType di = goReverse ? directionType::rev : directionType::fwd;

    backleftdrive.spin(di);
    frontleftdrive.spin(di);
    
    backrightdrive.spin(di);
    frontrightdrive.spin(di);
    ss.print("Rot: %.1f (%.1f)/ %.1f; Spd: %.1f; adj %.1f, err: %.1f", leftRot, backleftdrive.rotation(rotationUnits::deg), rightRot, speed, speed * err * kp, err);

    // GyroAdjust
    if (useGyro) {
      GyroAdjust = inertialSensor.rotation();
    }
    else {
      GyroAdjust = 0;
    }

    if (std::abs(GyroAdjust) > 0.5) {
      backrightdrive.setVelocity(speed + 1/2 * GyroAdjust, vex::percentUnits::pct);
      frontrightdrive.setVelocity(speed + 1/2 * GyroAdjust, vex::percentUnits::pct);

      backleftdrive.setVelocity(speed - 1/2 * GyroAdjust, vex::percentUnits::pct);
      frontleftdrive.setVelocity(speed - 1/2 * GyroAdjust, vex::percentUnits::pct);

      backrightdrive.spin(di);
      backleftdrive.spin(di);
      frontrightdrive.spin(di);
      frontleftdrive.spin(di);
    }

    vex::task::sleep(50);
  }

  // FINALE
  stopDriveTrain();
  ss.print("goStraight() ENDS");

}

/*
void goStraightWithGyro(double rotationsToGo, double baseSpeed, double minStartSpeed, double minEndSpeed, double kp) {
  // auto rotation using simplied PID (P-only) with help from inertial sensor
  // recommended baseSpeed is 30 (normal), 10 (slow)
  // kp = 0 (no P-adj), kp = 0.005 (small), kp = 0.1 (extremely large, ONLY for demo purpose)
  // requirement:
  // 1) inertialSensor takes values clock-wise (WARN: its reading is in range(0, 360), NO negative reading)
  // i.e. it starts at 0, a slight right reads a small positive value, a slight left reads a value slightly below 360
  // 2) leftDriveMotor and rightDriveMotor are both forward spinning
  //
  // PID algorithm in use:
  // the deviation in forward direction is computed as the diffence of rotations between left and right motors
  // P-only algorithm modifies the speed on both motors to suppress above deviation so that robot goes straight forward

  // INITIALIZATION
  leftDriveMotor.resetRotation(); 
  rightDriveMotor.resetPosition();

  // MOVEMENT LOOP
  while (leftDriveMotor.rotation(vex::rotationUnits::deg) < rotationsToGo * 360) {
    // set baseline speed
    double speed = baseSpeed;
    double leftRot = leftDriveMotor.rotation(vex::rotationUnits::deg);
    double rightRot = rightDriveMotor.rotation(vex::rotationUnits::deg);

    // for control purpose, we focus on err
    double err = inertialSensor.heading();
    if (err > 180.0) {
      err = 360.0 - err;
    }
    
    // compute "effective" speed depending on movement phases
    // 1) start (ramp-up) phase
    if (leftRot < 360) {
      speed = std::max(minStartSpeed, baseSpeed * leftRot / 360.0);
    }

    // 2) normal phase

    // 3) end (approaching) phase
    if (rotationsToGo * 360 - leftRot < 360) {
      speed = std::max(minEndSpeed, baseSpeed * (rotationsToGo * 360 - leftRot < 360)/360.0);
    }

    // drive the motors using equal+adj speeds in same direction
    leftDriveMotor.setVelocity(speed - speed * err * kp, vex::percentUnits::pct);
    rightDriveMotor.setVelocity(speed + speed * err * kp, vex::percentUnits::pct);

    leftDriveMotor.spin(directionType::fwd);
    rightDriveMotor.spin(directionType::fwd);
    ss.print("Rot: %f / %f; Speed: %f; adj %f, err: %f", leftRot, rightRot, speed, speed * err * kp, err);
    vex::task::sleep(10);
  }

  // FINALE
  leftDriveMotor.stop();
  rightDriveMotor.stop();
  ss.print("goStraightWithGryo() ENDS");

}
*/

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  ss.print("Hello, 47114A! Program to start.\n");

}


void autonomous( void ) {
  double kp = 0.0;
  double dist = 4 * 24;
  double backoutDist = 1.45 * 24;
  double dist2 = 0.5 * 24;
  ss.print("Go straight ahead for %4.1f inches, PID kp=%4.1f\n", dist, kp);

  double intakeSpeed = 60;

  leftintake.setVelocity(intakeSpeed, percentUnits::pct);
  rightintake.setVelocity(intakeSpeed, percentUnits::pct);

  leftintake.spin(directionType::rev);
  rightintake.spin(directionType::rev);
  goStraight(computeRotationsFromDistance(dist), 40, 10, 10, kp, false, true);
  
  vex::task::sleep(500);

  leftintake.stop(brakeType::hold);
  rightintake.stop(brakeType::hold);

  ss.print("Go reverse");
  goStraight(computeRotationsFromDistance(backoutDist), 40, 10, 10, kp, true, true);
  //vex::task::sleep(500);

  ss0.print("Make left turn");
  makeTurn(20, 5, 0, true);
  //vex::task::sleep(500);

  // ss0.print("Make right turn");
  // makeTurn(15, 5, 0, false);
  // vex::task::sleep(1000);

  // ss0.print("Make 2nd left turn");
  //   makeTurn(15, 5, 0, true);
  // vex::task::sleep(500);

  // ss0.print("Make 2nd right turn");
  // makeTurn(15, 5, 0, false);
  // vex::task::sleep(1000);

  // 2nd stage go straight (to left)
  ss.print("Go straight (to left)");
  goStraight(computeRotationsFromDistance(dist2), 40, 10, 10, 0.0, false, true);

  leftintake.spin(directionType::rev);
  rightintake.spin(directionType::rev);
  
  vex::task::sleep(300);

  leftintake.setBrake(brakeType::hold);
  rightintake.setBrake(brakeType::hold);
  ss.print("START: lift up");
  lift.spinFor(vex::directionType::fwd, 0.75, vex::rotationUnits::rev);
  vex::task::sleep(1000);

  // back off
  ss.print("START: go back again");
  goStraight(computeRotationsFromDistance(dist2), 40, 10, 10, kp, true /*reverse*/, true);
  ss.print("END: go back again");

  // 180 degree turn as two 90 left turns
  makeTurn(20, 5, 0, true); 
  makeTurn(20, 5, 0, true);


  // push balls out
  leftintake.spin(directionType::fwd);
  rightintake.spin(directionType::fwd);

  leftintake.setBrake(brakeType::hold);
  rightintake.setBrake(brakeType::hold);
  


  ss.print("DONE");
  ss.print("dist for one rot: %.2f", computeDistanceForOneRotation());
}


void usercontrol( void ) {
  ss.print("User control not impl yet");
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}
