/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Chloe Ning                                       */
/*    Created:      Fri Dec 25 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "utils/nutils.hpp"

using namespace vex;
competition Competition;

// use heading(), data range [0, 360]. so calibrate to 0 means the initial reading can be 365.5 !

double x = 0.5;


auto ss = ScrollingScreen();


void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  // Brain.Screen.print("Hello, 47114A! Program to start.\n");
  //lift.stop(vex::brakeType::hold);
  claw.stop(vex::brakeType::hold);
}


void autonomous(void) {
  // auto rotation using PID and inertial sensor
  inertialSensor.calibrate();

  while (inertialSensor.isCalibrating()) {
    vex::wait(50, timeUnits::msec);
  }


  inertialSensor.resetRotation();
  leftDriveMotor.resetPosition();
  rightDriveMotor.resetPosition();

  double origSpeed = 10;
  double speed = 0.0;

  ss.print("Start inertia sensor run \n");
  ss.print("Initial: %4.1f \n", inertialSensor.rotation());

  while ((inertialSensor.heading() < 90.0) || (inertialSensor.heading() > 180)) {
    speed = origSpeed;
    // slow start

    double r = inertialSensor.heading();

    if (r > 180) {
      r = r - 360;
    }

    if (r < 10.0) {
      speed = origSpeed * std::max(0.3, r / 10.0);

    }

    if (r > 80.0) {
      speed = origSpeed * (1.0 - (r - 80.0)/ 10.0) * 3/4;
    }

    leftDriveMotor.setVelocity(speed, vex::percentUnits::pct);

    rightDriveMotor.setVelocity(speed, vex::percentUnits::pct);  // not moving !!!

    leftDriveMotor.spin(directionType::fwd);
    rightDriveMotor.spin(directionType::rev);

    ss.print("Rotate %4.1f , %4.1f\n", r, speed);
    
    vex::task::sleep(1);
  } // while

  Brain.Screen.print("Stopped");
  leftDriveMotor.stop();
  rightDriveMotor.stop();
}

void turnleft(void) {
  // auto rotation using PID and inertial sensor
  inertialSensor.calibrate();

  while (inertialSensor.isCalibrating()) {
    vex::wait(50, timeUnits::msec);
  }


  inertialSensor.resetRotation();
  leftDriveMotor.resetPosition();
  rightDriveMotor.resetPosition();

  double origSpeed = 10;
  double speed = 0.0;
  Brain.Screen.clearScreen();
  Brain.Screen.print("Start inertia sensor run \n");
  Brain.Screen.print("Initial: %4.1f \n", inertialSensor.rotation());
  Brain.Screen.newLine();

  while ((inertialSensor.heading() < 90.0) || (inertialSensor.heading() > 180)) {
    speed = origSpeed;
    // slow start

    double r = 360 - inertialSensor.heading(); // this is to make sure we get a small, negative number.

    if (r > 180) {
      r = r - 360;
    }

    if (r < 10.0) {
      speed = origSpeed * std::max(0.3, r / 10.0);

    }

    if (r > 80.0) {
      speed = origSpeed * (1.0 - (r - 80.0)/ 10.0) * 1.3;
    }

    leftDriveMotor.setVelocity(speed, vex::percentUnits::pct);

    rightDriveMotor.setVelocity(speed, vex::percentUnits::pct);  // not moving !!!

    leftDriveMotor.spin(directionType::rev);
    rightDriveMotor.spin(directionType::fwd);

    Brain.Screen.print("Rotate %4.1f , %4.1f\n", r, speed);
    Brain.Screen.newLine();
    vex::task::sleep(1);
  } // while

  leftDriveMotor.stop();
  rightDriveMotor.stop();
}

void autonomouswithdt(void) {
  // auto rotation using PID and inertial sensor
  inertialSensor.calibrate();
  Brain.Screen.print("started started started");

  while (inertialSensor.isCalibrating()) {
    vex::wait(50, timeUnits::msec);
  }


  inertialSensor.resetRotation();
  leftDriveMotor.resetPosition();
  rightDriveMotor.resetPosition();

  double origSpeed = 30;
  double speed = 0.0;
  Brain.Screen.clearScreen();
  Brain.Screen.print("Start inertia sensor run \n");
  Brain.Screen.print("Initial: %4.1f \n", inertialSensor.rotation());

  while ((inertialSensor.heading() < 90.0) || (inertialSensor.heading() > 180.0)) {
    speed = origSpeed;
    // slow start
    if (inertialSensor.heading() < 10.0) {
      speed = origSpeed * std::max(0.5, inertialSensor.rotation() / 10.0);

    }

    if (inertialSensor.heading() > 80.0) {
      speed = origSpeed * (1.0 - (inertialSensor.rotation() - 80.0)/ 10.0);
    }

    dt.setDriveVelocity(speed, vex::percentUnits::pct);

    dt.turn(right);

    Brain.Screen.print("Rotate %4.1f \n", inertialSensor.heading());
    vex::task::sleep(50);
  } // while

  leftDriveMotor.stop();
  rightDriveMotor.stop();
}



void autonomousGo(void ) {
  Brain.Screen.print("start auton");
  leftDriveMotor.resetRotation(); 
  rightDriveMotor.resetPosition();

  while (leftDriveMotor.rotation(vex::rotationUnits::deg) < 4 * 360) {
    double err = leftDriveMotor.rotation(vex::rotationUnits::deg) - rightDriveMotor.rotation(vex::rotationUnits::deg);
    double kp = 0.005;  // 0.005 is good

    Brain.Screen.setCursor(1,0);
    Brain.Screen.print("L: %f, R: %f, err: %f", leftDriveMotor.rotation(vex::rotationUnits::deg),\
       rightDriveMotor.rotation(vex::rotationUnits::deg), err);
    
    double speed = 30;
    double origSpeed = 30;

    if (leftDriveMotor.rotation(vex::rotationUnits::deg) < 360) {
      speed = origSpeed * leftDriveMotor.rotation(vex::rotationUnits::deg)/ 360.0;

      if (speed < 10) {
        speed = 10;
      }
    }

    if (leftDriveMotor.rotation(vex::rotationUnits::deg) > 2 * 360) {
      double ratio = (3*360 - leftDriveMotor.rotation(vex::rotationUnits::deg)) / (360);

      speed = ratio * origSpeed;
    }


    leftDriveMotor.setVelocity(speed, vex::percentUnits::pct);

    rightDriveMotor.setVelocity(speed + speed * err * kp, vex::percentUnits::pct);

    leftDriveMotor.spin(directionType::fwd);
    rightDriveMotor.spin(directionType::fwd);

    vex::task::sleep(50);
  }

  leftDriveMotor.stop();
  rightDriveMotor.stop();
}



void autonomous_1( void ) {
  // copied from our Robot Mesh code used in Roslyn event
  for (int i = 0; i < 3; ++i) {
    dt.driveFor(vex::directionType::fwd, 60, vex::distanceUnits::in);
    dt.driveFor(vex::directionType::rev, 60, vex::distanceUnits::in);
    vex::task::sleep(3000);
  }
}

void usercontrol( void ) {
  int pinch = 0;

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

void motortest( void ) {
  rightDriveMotor.spinFor(vex::directionType::fwd, 80, vex::rotationUnits::deg);
  Brain.Screen.print("one done");
  vex::task::sleep(100);
  leftDriveMotor.spinFor(vex::directionType::fwd, 100, vex::rotationUnits::deg);
  Brain.Screen.print("two done");
}


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton();

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}