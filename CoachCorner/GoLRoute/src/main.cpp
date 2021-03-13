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
#include <algorithm>
#include "vex.h"

#include "../../../Utils/nutils.hpp"

using namespace vex;
competition Competition;

// use heading(), data range [0, 360]. so calibrate to 0 means the initial reading can be 365.5 !
auto ss0 = ScrollingScreen<int>(0, 3);
auto ss = ScrollingScreen<int>(3, 5);


double computeDistanceForOneRotation() {
  // return the wheel travel distance (in inch) when motor spins one rotation
  double wheelDiameter = 4.0; // omni "green" wheel
  double wheelToDriveGearRatio = 1.0;
  double motorGearRatio = 18.0; // standard gear in-use, 200 rpm
  
  return 3.14159265 * wheelDiameter / wheelToDriveGearRatio / motorGearRatio;
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


void makeTurn(double baseSpeed, double minStartSpeed, double multiplierStartSpeed, double minEndSpeed, double multiplierEndSpeed, bool turnLeft) {
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

  while (inertialSensor.isCalibrating()) {
    vex::wait(50, timeUnits::msec);
  }

  inertialSensor.resetRotation();

  resetDriveTrainRotation();

  double speed = 0.0;

  ss.print("Initial: %.1f \n", inertialSensor.rotation());

  // MOVEMENT LOOP
  while ((inertialSensor.heading() < 90.0) || (inertialSensor.heading() > 180)) {
    // set baseline speed
    speed = baseSpeed;

    // for control purpose, we focus on r, not the raw reading from inertial sensor
    // r: start from 0; orientation free 
    //  i.e. 
    //  r= 90 means 
    //  -- it made 90 deg LEFT if turnLeft=True; 
    //  -- it made 90 deg RIGHT if turnLeft=False.

    double r = 360 - inertialSensor.heading(); 

    if (r > 180) {
      r = r - 360;
    }

    // compute "effective" speed depending on movement phases
    // 1) start (ramp-up) phase
    if (r < 10.0) {
      speed = std::max(minStartSpeed, baseSpeed * multiplierStartSpeed * (r / 10.0));
    }

    // 2) normal phase

    // 3) end (approaching) phase
    if (r > 80.0) {
      speed = std::max(minEndSpeed, baseSpeed * multiplierEndSpeed * (1.0 - (r - 80.0)/ 10.0));
    }

    // drive the motors using equal speeds but opposite direction to turn
    backleftdrive.setVelocity(speed, vex::percentUnits::pct);
    frontleftdrive.setVelocity(speed, vex::percentUnits::pct);

    backrightdrive.setVelocity(speed, vex::percentUnits::pct);
    frontrightdrive.setVelocity(speed, vex::percentUnits::pct);

    backleftdrive.spin(directionType::rev);
    frontleftdrive.spin(directionType::rev);
    
    backrightdrive.spin(directionType::fwd);
    frontrightdrive.spin(directionType::fwd);

    ss.print("Rotate %4.1f , %4.1f\n", r, speed);
    vex::task::sleep(10);
  } // while

  // FINALE
  stopDriveTrain();

  ss.print("makeTurn() ENDS");
}


void goStraight(double rotationsToGo, double baseSpeed, double minStartSpeed, double minEndSpeed, double kp) {
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

  // INITIALIZATION
  resetDriveTrainRotation();

  // MOVEMENT LOOP
  while (backleftdrive.rotation(vex::rotationUnits::deg) < rotationsToGo * 360) {
    // set baseline speed
    double speed = baseSpeed;
    double leftRot = backleftdrive.rotation(vex::rotationUnits::deg);
    double rightRot = backrightdrive.rotation(vex::rotationUnits::deg);

    // for control purpose, we focus on err
    double err = leftRot - rightRot;
    if (err > 180.0) {
      err = 360.0 - err;
    }
    
    // compute "effective" speed depending on movement phases
    // start (ramp-up) phase
    if (leftRot < 360) {
      speed = std::max(minStartSpeed, baseSpeed * leftRot / 360.0);
    }
    else if (rotationsToGo * 360 - leftRot < 360) { // end (approaching) ramp down phase
      speed = std::max(minEndSpeed, baseSpeed * (rotationsToGo * 360 - leftRot < 360)/360.0);
    }
    else { // normal  phase
      speed = baseSpeed;
    }

    // drive the motors using equal+adj speeds in same direction
    backleftdrive.setVelocity(speed - speed * err * kp, vex::percentUnits::pct);
    frontleftdrive.setVelocity(speed - speed * err * kp, vex::percentUnits::pct);

    backrightdrive.setVelocity(speed + speed * err * kp, vex::percentUnits::pct);
    frontrightdrive.setVelocity(speed + speed * err * kp, vex::percentUnits::pct);

    backleftdrive.spin(directionType::fwd);
    frontleftdrive.spin(directionType::fwd);
    
    backrightdrive.spin(directionType::fwd);
    frontrightdrive.spin(directionType::fwd);
    ss.print("Rot: %.1f / %.1f; Spd: %.1f; adj %.1f, err: %.1f", leftRot, rightRot, speed, speed * err * kp, err);
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
  double dist = 1.0;
  ss.print("Go straight ahead for %4.1f inches, PID kp=%4.1f\n", dist, kp);

  goStraight(1, 50, 20, 20, 0.0);
  
  vex::task::sleep(2000);

  
  goStraight(3, 50, 20, 20, 0.0);
  
  vex::task::sleep(2000);

  
  goStraight(10, 50, 20, 20, 0.0);
  
  vex::task::sleep(2000);


  
  // goStraight(20, 50, 20, 20, 0.0);
  
  // vex::task::sleep(2000);
  
  ss.print("Make left turn");
  makeTurn(50, 10, 1.1, 10, 1.1, true);
  ss.print("DONE");
  ss.print("dist for one rot: %.f", computeDistanceForOneRotation());
}


void usercontrol( void ) {
  ss.print("User control not impl yet");
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}
