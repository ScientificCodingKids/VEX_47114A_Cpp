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

auto ss = ScrollingScreen<int>();


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
  leftDriveMotor.resetPosition();
  rightDriveMotor.resetPosition();

  double speed = 0.0;

  ss.print("Initial: %4.1f \n", inertialSensor.rotation());
  
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
    leftDriveMotor.setVelocity(speed, vex::percentUnits::pct);
    rightDriveMotor.setVelocity(speed, vex::percentUnits::pct);

    leftDriveMotor.spin(directionType::rev);
    rightDriveMotor.spin(directionType::fwd);

    ss.print("Rotate %4.1f , %4.1f\n", r, speed);
    vex::task::sleep(10);
  } // while

  // FINALE
  leftDriveMotor.stop();
  rightDriveMotor.stop();
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
  leftDriveMotor.resetRotation(); 
  rightDriveMotor.resetRotation();

  // MOVEMENT LOOP
  while (leftDriveMotor.rotation(vex::rotationUnits::deg) < rotationsToGo * 360) {
    // set baseline speed
    double speed = baseSpeed;
    double leftRot = leftDriveMotor.rotation(vex::rotationUnits::deg);
    double rightRot = rightDriveMotor.rotation(vex::rotationUnits::deg);

    // for control purpose, we focus on err
    double err = leftRot - rightRot;
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
  ss.print("goStraight() ENDS");

}


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


void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  ss.print("Hello, 47114A! Program to start.\n");

}


void autonomous( void ) {
  // copied from our Robot Mesh code used in Roslyn event
  // todo
}


void usercontrol( void ) {
  int pinch = 0;
  double x = 0.5;

  while (1) { // put all our code within this indefinite loop
  
    if (rc.ButtonA.pressing()) {
      x = 0.8;
    }

    if (rc.ButtonLeft.pressing()) {
      x = 0.5;
    }

    if (rc.ButtonUp.pressing()) {
      pinch = 1;
    } 
    if (rc.ButtonR1.pressing()) {
      lift.spin(vex::directionType::fwd);
      // leftdownlift.setVelocity(7, vex::percentUnits::pct);
      // rightdownlift.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonR2.pressing()) {
      lift.spin(vex::directionType::rev);
      //  rightdownlift.spin(vex::directionType::rev);
    }
    else {
      lift.stop(vex::brakeType::hold);
  //      leftdownlift.stop(vex::brakeType::hold);
    }
 
    double leftMotorSpeed = rc.Axis3.position(vex::percentUnits::pct) * x;
    double rightMotorSpeed = rc.Axis2.position(vex::percentUnits::pct) * x;
 
    double clawMoveSpeed = 20;
    double clawPinchSpeed = 0.5;  // force claw to pinch tightly

    if (fabs(leftMotorSpeed) > 5.0) {
      leftDriveMotor.setVelocity(leftMotorSpeed, vex::velocityUnits::pct);
      leftDriveMotor.spin(fwd);
    }
    else {
      leftDriveMotor.stop(vex::brakeType::hold);
    }

    if (fabs(rightMotorSpeed) > 5.0) {
      rightDriveMotor.setVelocity(rightMotorSpeed, vex::velocityUnits::pct);
      rightDriveMotor.spin(fwd);
    }
    else {
      rightDriveMotor.stop(vex::brakeType::hold);
    }


    if (rc.ButtonL1.pressing()) {
      claw.setVelocity(clawMoveSpeed, vex::percentUnits::pct);
      claw.spin(vex::directionType::fwd);
    }
    else if (rc.ButtonL2.pressing()) {
      claw.setVelocity(clawMoveSpeed, vex::percentUnits::pct);
      claw.spin(vex::directionType::rev);
    }
    else {
      if (pinch == 1) {
        claw.setVelocity(clawPinchSpeed, vex::percentUnits::pct);
        claw.spin(vex::directionType::fwd);
      }
      else { 
        claw.stop(vex::brakeType::brake);
      }
    }

    vex::task::sleep(20); //Sleep the task for a short amount of time to prevent wasted resources. 
  }
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  
}
