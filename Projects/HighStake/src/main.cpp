/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Chloe Ning                                                */
/*    Created:      9/29/2024, 7:22:13 PM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>
#include <algorithm>

using namespace vex;
using namespace std;


void goStraight(Coord& loc, double dist, vex::directionType dt, double origSpeed, double kp=0.004, vex::brakeType bt=coast, double wheelDiameter=4.0) {
  leftDrive.resetPosition();
  rightDrive.resetPosition();

  double tgtHeading = inertialSensor.heading();
  double distToGo = dist; // distance more to travel
  double distTravelled = 0.0; // distance already traveled
  double finalSpeed = 10; // capping speed
  double speed = origSpeed; // max speed
  double const adaptiveInterval = 10; // slow down/speed up interval
  double prevRot = leftDrive.position(vex::rotationUnits::deg); // amount of rotation at the last run through
  double changedRotations = 0; // rotations passed since last loop
  double dx = 0; // change in x coordinate since last loop
  double dy = 0; // change in y coordinate since last loop

  while (distToGo > 0) {
    double h = inertialSensor.heading();

    double hE = h - tgtHeading;

    if (hE < -270) hE = hE + 360;
    if (hE > 270) hE = hE - 360;
    // now -90 <= hE <= 90
    if (hE < -15) hE = -15;
    if (hE > 15) hE = 15;
    // after truncation, now -15 <= hE <= 15; kp * maxHE = 0.004 * 15 = 0.06

    speed = origSpeed;
    if (distTravelled < adaptiveInterval) speed = origSpeed * distTravelled / adaptiveInterval;
    if (distToGo < adaptiveInterval) speed = origSpeed * (1 - (adaptiveInterval - distToGo) / adaptiveInterval);
    if (speed < finalSpeed) speed = finalSpeed;

    double speedAdj = kp * hE; 

    speedAdj = max(-0.1, min(0.1, speedAdj));
    double lowSpd = speed * (1. - speedAdj);
    double highSpd = speed * (1. + speedAdj);

    leftDrive.setVelocity(dt == fwd? lowSpd: highSpd, pct);
    rightDrive.setVelocity(dt == fwd? highSpd: lowSpd, pct); 
    
    leftDrive.spin(dt);
    rightDrive.spin(dt);

    vex::task::sleep(10);

    // update coordinates and re-estimate the progress
    double rot = abs(leftDrive.position(vex::rotationUnits::deg)) / 360. * (wheelDiameter * M_PI);  // assuming diameter 4 inch???

    changedRotations = rot - prevRot; // need scale, deg => inch (really??)

    distTravelled += changedRotations;
    distToGo = dist - distTravelled;

    Brain.Screen.printAt(3, 25, "H=%.1f, Spd=%.1f, %.1f; rot=%.1f, toGo=%.1f", h, lowSpd, highSpd, rot, distToGo);

    dx = changedRotations * cos( (90-h)/ M_PI);
    dy = changedRotations * sin( (90-h)/ M_PI);

    // prepare for next iteration
    prevRot = rot;
    
  }

  leftDrive.stop(bt);
  rightDrive.stop(bt);

  // update location
  loc.x += dx;
  loc.y += dy;
}

void pre_auton(void) {
  mogomech.set(true);  // release

  inertialSensor.calibrate();

  while (inertialSensor.isCalibrating()) {
    // busy waiting
    vex::task::sleep(50);
  }
 
}

void my_func() {
    intake.spin(directionType::fwd);

}

void my_go_straight_10() {
    // this demonstrates how to wrap a parameter-taking function into param-less function so that it can be used by thread API
    leftDrive.spinFor(5, timeUnits::sec);
    rightDrive.spinFor(5, timeUnits::sec);
}

void move_back_left() {

}

void move_back_right() {}

void autonomous1(void) {
// focus: move fast and suck in rings reliably
// yes, high speed movement can pick up 2 rings along its way.
  intake.setVelocity(70, pct);
  intake.spin(fwd);

  Coord loc(0, 0);

  goStraight(loc, 40, fwd, 80);
  Brain.Screen.printAt(0, 30, "Fast move to {%.1f}: {%.1f}", loc.x, loc.y);
  goStraight(loc, 1.5, fwd, 30);
  Brain.Screen.printAt(0, 35, "Slow move to {%.1f}: {%.1f}", loc.x, loc.y);


  goStraight(loc, 0.5, vex::directionType::rev, 80.);

}


void task_go_in_autom() {
  Coord loc(0, 0);
  Brain.Screen.printAt(0, 10, "XXXXXXXX");
  goStraight(loc, 25., vex::directionType::rev, 50.);
  goStraight(loc, 10, directionType::fwd, 40);

}

void task_grab_base_in_autom() {
    task::sleep(2000);
    mogomech.set(false);

}

void autonomous(void) {
// focus: move fast and grab goal reliably
  //return;

  Coord loc(0, 0);

  //goStraight(loc, 40, vex::directionType::rev, 80);
  //Brain.Screen.printAt(0, 30, "Fast move to {%.1f}: {%.1f}", loc.x, loc.y);
  //goStraight(loc, 1.5, vex::directionType::rev, 30);
  //Brain.Screen.printAt(0, 5, "Slow move to {%.1f}: {%.1f}", loc.x, loc.y);

  //mogomech.set(false);

  //goStraight(loc, 30., vex::directionType::rev, 50.);

  //mogomech.set(false);

  //goStraight(loc, 5., vex::directionType::fwd, 5.);

  vex::thread t1(task_go_in_autom);
  vex::thread t2(task_grab_base_in_autom);

  Brain.Screen.printAt(5, 5, "go: %d, grab: %d", t1.get_id(), t2.get_id());

  task::sleep(2000);
}


void autonomous0(void) {
  // example code to use multiple threads to achieve concurrency
  int count = 0;

    // C++ allows us to pass function name (actually the address of the func) as argument to thread constructor function
    
    vex::thread t(my_go_straight_10);  // robot will immediate execute the code as in goStraight() -- in a new thread B
    task::sleep(100);
    vex::thread t2(my_func);  // associate func to a new thread (A); my_func is executed immediately upon creation of thread A

    while (1) {
        Brain.Screen.printAt(10, 30, "Hello from main %d", count++);

        // Allow other tasks to run
        vex::this_thread::sleep_for(100);
    }

}


void usercontrol(void) {
  // User control code here, inside the loop

  leftDrive.setVelocity(30, percentUnits::pct); 
  rightDrive.setVelocity(30, percentUnits::pct);
  int leftJoyPos;
  int rightJoyPos;
  int leftDriveSpeed;
  int rightDriveSpeed;
  directionType leftDriveDir = directionType::fwd;
  directionType rightDriveDir = directionType::fwd;

  bool getGoal = false;

  while (1) {
    leftJoyPos = rc.Axis3.position();
    rightJoyPos = rc.Axis2.position();
    leftDriveSpeed = abs(leftJoyPos * 0.85);
    rightDriveSpeed = abs(rightJoyPos * 0.85);

    if (leftJoyPos > 5) {
      leftDriveDir = directionType::fwd;
      frontleftdrive.setVelocity(leftDriveSpeed, velocityUnits::pct);
      backleftdrive.setVelocity(leftDriveSpeed, velocityUnits::pct);
    }
    else if (leftJoyPos < -5) {
      leftDriveDir = directionType::rev;
      frontleftdrive.setVelocity(leftDriveSpeed, velocityUnits::pct);
      backleftdrive.setVelocity(leftDriveSpeed, velocityUnits::pct);
    }
    else {
      frontleftdrive.setVelocity(0, velocityUnits::pct);
      backleftdrive.setVelocity(0, velocityUnits::pct);
    }

    if (rightJoyPos > 5) {
      rightDriveDir = directionType::fwd;
      frontrightdrive.setVelocity(rightDriveSpeed, velocityUnits::pct);
      backrightdrive.setVelocity(rightDriveSpeed, velocityUnits::pct);
    }
    else if (rightJoyPos < -5) {
      rightDriveDir = directionType::rev;
      frontrightdrive.setVelocity(rightDriveSpeed, velocityUnits::pct);
      backrightdrive.setVelocity(rightDriveSpeed, velocityUnits::pct);
    }
    else {
      frontrightdrive.setVelocity(0, velocityUnits::pct);
      backrightdrive.setVelocity(0, velocityUnits::pct);
    }

    frontleftdrive.spin(leftDriveDir);
    frontrightdrive.spin(rightDriveDir);
    backleftdrive.spin(leftDriveDir);
    backrightdrive.spin(rightDriveDir);

    intake.setVelocity(50, velocityUnits::pct);
    conveyor.setVelocity(50, velocityUnits::pct);
    //rs.print("Intake SPIN at %.1f \n", currSpeed);
    
    if (rc.ButtonUp.pressing()) {
      intake.spin(directionType::fwd);
    }
    else if (rc.ButtonDown.pressing()) {
      intake.spin(directionType::rev);
    }
    else if (rc.ButtonX.pressing()) {
      intake.stop(brakeType::coast);
    }

    if (rc.ButtonLeft.pressing()) {
      conveyor.spin(directionType::fwd);
    }
    else if (rc.ButtonRight.pressing()) {
      conveyor.spin(directionType::rev);
    }
    else if (rc.ButtonY.pressing()) {
      conveyor.stop(brakeType::coast);
    }

    if (rc.ButtonL1.pressing()) {
     mogomech.set(true);
    }
    else if (rc.ButtonL2.pressing()) {
      mogomech.set(false);
    }

    if (rc.ButtonUp.pressing()) {
      leftDrive.spin(directionType::rev);
      rightDrive.spin(directionType::rev);
    }
    if (rc.ButtonDown.pressing()) {
      leftDrive.stop();
      rightDrive.stop();
    }

    if (rc.ButtonR1.pressing() && !getGoal) {
      getGoal = true;
      autonomous();
      getGoal = false;
    }

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

