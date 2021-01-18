/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Chloe Ning                                       */
/*    Created:      Wed Jan 22 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------xsssssssssss------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

competition Competition;

double x = 0.5;

void pre_auton( void ) {
  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  Brain.Screen.print("Hello, 47114A! Program to start.\n");
  //lift.stop(vex::brakeType::hold);
  claw.stop(vex::brakeType::hold);
}

void autonomous(void ) {
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

void autonomouswithgyro(void ) {
  Brain.Screen.print("start auton");
  leftDriveMotor.resetRotation();
  rightDriveMotor.resetPosition();
  inertialSensor.calibrate();

  while (inertialSensor.isCalibrating()) {
    vex::wait(50, timeUnits::msec);
  }

  inertialSensor.resetRotation();
  leftDriveMotor.resetPosition();
  rightDriveMotor.resetPosition();

  Brain.Screen.print("Start inertia sensor run \n");
  Brain.Screen.print("Initial: %4.1f \n", inertialSensor.rotation());

  while (leftDriveMotor.rotation(vex::rotationUnits::deg) < 2 * 360) {
    double err = inertialSensor.heading();
    double kp = 0.01;  // 0.005 is good
    double speed = 30;
    double origSpeed = 30;
    
    Brain.Screen.setCursor(1,0);
    Brain.Screen.print("L: %f, R: %f, err: %f", leftDriveMotor.rotation(vex::rotationUnits::deg),\
       rightDriveMotor.rotation(vex::rotationUnits::deg), err);
    

    if (leftDriveMotor.rotation(vex::rotationUnits::deg) < 180) {
      if (speed < 10) {
        speed = 10;
      }
      else {
        speed = origSpeed * leftDriveMotor.rotation(vex::rotationUnits::deg)/ 360.0;
      }
    }

    if (leftDriveMotor.rotation(vex::rotationUnits::deg) > 360) {
      double ratio = (3*360 - leftDriveMotor.rotation(vex::rotationUnits::deg)) / (360);

      speed = ratio * origSpeed;
    }


    leftDriveMotor.setVelocity(speed, vex::percentUnits::pct);

    rightDriveMotor.setVelocity(speed + speed * kp, vex::percentUnits::pct);

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


int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Brain.Screen.print("Get ready to start!");
  Competition.autonomous( autonomouswithgyro );
  Competition.drivercontrol( usercontrol );

  pre_auton();

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}