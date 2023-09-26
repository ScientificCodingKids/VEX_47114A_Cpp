#include "vex.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include "../../../Utils/nutils.hpp"
//#include "../../../Utils/motion.hpp"
#include "../../../Utils/XDrive.hpp"
#include <tuple>
//#include <vex_task.h>

using namespace vex;
using namespace std;

competition Competition;
XDriveRobot robot(backleftdrive, backrightdrive, frontleftdrive, frontrightdrive, Brain, inertialSensor, flywheel, expander, intake, roller);
RollingScreen rs(robot.Brain.Screen);

bool AT_HOME = false;  // if at home, need calibrate in auto() and drive() as no pre_auton() is called

int straight180() {
    robot.goStraight(24, directionType::fwd, 180, 60);
    return 0;
}

int spinForever() {
    robot.flywheel.spinFor(directionType::fwd, 20, rotationUnits::rev);
    return 0;
}

void FarTeam(XDriveRobot& robot) {
    robot.goStraight(20, directionType::fwd, 270, 60);
    robot.move(-50, -50, 50, 50);
    vex::task::sleep(1000);
    robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    robot.roller.stop(vex::brakeType::coast);

/*
    robot.goStraight(5, directionType::fwd, 180, 70);
    robot.makeTurn(45, true);
    robot.goStraight(2.1 * 24 + 1, directionType::fwd, 135, 80);

    robot.flywheel.spin(directionType::fwd, 62, velocityUnits::pct);
    vex::task::sleep(3000);
    robot.intake.spin(directionType::fwd, 30, velocityUnits::pct);
    */
    return;
}

void ShortTeam(XDriveRobot& robot) {
    robot.move(-50, -50, 50, 50);
    vex::task::sleep(1000);
    robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    robot.roller.stop(vex::brakeType::coast);

    robot.goStraight(5, directionType::fwd, 180, 50);
    robot.makeTurn(45, true);
    robot.goStraight(2.1 * 24, directionType::fwd, 135, 80);

    robot.flywheel.spin(directionType::fwd, 60, velocityUnits::pct);
    vex::task::sleep(3000);
    robot.intake.spin(directionType::fwd, 30, velocityUnits::pct);
    return;
}

void PrimitiveTeam(XDriveRobot& robot) {
    robot.move(-50, -50, 50, 50);
    vex::task::sleep(1000);
    robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    robot.roller.stop(vex::brakeType::coast);
}

void FarSkillsDiagonal(XDriveRobot& robot) {

    // getting first roller (directly in front)
    robot.move(-50, -50, 50, 50);
    vex::task::sleep(1000);
    robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1400);
    robot.roller.stop(vex::brakeType::coast);
    
    // move to second roller
    robot.goStraight(7, directionType::fwd, 180, 70, 0.05);
    robot.makeTurn(90, true); // turn to push dir
    robot.goStraight(15, directionType::fwd, 180, 70, 0.05); // push away
    robot.goStraight(8, directionType::fwd, 90, 70, 0.05); // go forwards a little
    robot.goStraight(5, directionType::fwd, 0, 70, 0.05); // move left
    robot.goStraight(5, directionType::fwd, 90, 70, 0.05); // 

    // getting the second roller
    robot.move(-50, -50, 50, 50);
    vex::task::sleep(700);
    robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1200);
    robot.roller.stop(vex::brakeType::coast);

    // move to throwing position
    robot.goStraight(10, directionType::fwd, 270, 70);
    robot.makeTurn(180, true);
    robot.goStraight(2 * 24 - 5, directionType::fwd, 270, 70);
    robot.goStraight(5, directionType::fwd, 90, 70);
    robot.makeTurn(108, false, 40, 3);

    robot.flywheel.spin(directionType::fwd, 53, velocityUnits::pct);
    vex::task::sleep(2500);
    robot.intake.spin(directionType::fwd, 40, velocityUnits::pct);

    vex::task::sleep(6000);
    robot.flywheel.stop(brakeType::coast);
    robot.intake.stop(brakeType::coast);

    // move to long term diagonal position
    robot.makeTurn(90, false, 40, 3);
    robot.goStraight(21, directionType::fwd, 90, 70);
    robot.goStraight(3.2 * 23.5, vex::directionType::fwd, 225, 80);
    
    robot.makeTurn(179, true, 60, 1);
    //rs.print("before %d", 7);
    robot.goStraight(7, vex::directionType::fwd, 180, 70);
    //rs.print("AFTER %d", 7);

    // get third roller
    task t1(straight180);
    vex::task::sleep(800);
    t1.stop();
    robot.stop(brakeType::coast);

    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    robot.roller.stop(vex::brakeType::coast);

    // move to fourth roller    
    robot.goStraight(17, directionType::fwd, 0, 70, 0.05);
    robot.makeTurn(270, true);
    robot.goStraight(15, directionType::fwd, 270, 70, 0.05);

    // get fourth roller
    robot.move(-50, -50, 50, 50);
    vex::task::sleep(700);
    robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    robot.roller.stop(vex::brakeType::coast);

    // move to optimal expansion position
    robot.goStraight(12, vex::directionType::fwd, 90, 70);
    robot.goStraight(12, vex::directionType::fwd, 0, 70);
    robot.makeTurn(135, false, 60, 1);

    // expand
    robot.expander.spin(vex::directionType::fwd, 70, vex::velocityUnits::pct);
    vex::task::sleep(7000);
    robot.expander.stop(vex::brakeType::coast);
    
}

void FarSkills(XDriveRobot& robot) {
    robot.move(-50, -50, 50, 50);
    vex::task::sleep(1000);
    robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    robot.roller.stop(vex::brakeType::coast); 
    
    robot.goStraight(17, directionType::fwd, 180, 50, 0.05);
    robot.makeTurn(90, true);
    //robot.intake.spin(directionType::fwd, 50, velocityUnits::pct);
    robot.goStraight(15, directionType::fwd, 90, 50, 0.05);

    robot.move(-50, -50, 50, 50);
    vex::task::sleep(700);
    robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    robot.roller.stop(vex::brakeType::coast);
    //robot.intake.stop(vex::brakeType::coast);

    robot.goStraight(2 * 24, directionType::fwd, 270, 50);
    robot.makeTurn(100, true, 40, 3);

    robot.flywheel.spin(directionType::fwd, 53, velocityUnits::pct);
    vex::task::sleep(2500);
    robot.intake.spin(directionType::fwd, 40, velocityUnits::pct);

    vex::task::sleep(7000);
    robot.flywheel.stop(brakeType::coast);
    robot.intake.stop(brakeType::coast);

    robot.makeTurn(90, false, 40, 3);
    robot.goStraight(5, directionType::fwd, 90, 50);
    robot.goStraight(2.75 * 23.5, directionType::fwd, 180, 70);
    robot.goStraight(26.5, directionType::fwd, 270, 50);
    robot.makeTurn(180, true);

    //robot.move(-50, -50, 50, 50);
    //vex::task::sleep(700);
    //robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    robot.roller.stop(vex::brakeType::coast);

    /*
    robot.goStraight(17, directionType::fwd, 0, 50, 0.05);
    robot.makeTurn(270, true);
    //robot.intake.spin(directionType::fwd, 50, velocityUnits::pct);
    robot.goStraight(15, directionType::fwd, 270, 50, 0.05);

    robot.move(-50, -50, 50, 50);
    vex::task::sleep(700);
    robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    robot.roller.stop(vex::brakeType::coast);
    */
}

void autonWithXD(XDriveRobot& robot) {

    robot.move(-50, -50, 50, 50);
    vex::task::sleep(1000);
    robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    robot.roller.stop(vex::brakeType::coast); 
    
    robot.goStraight(17, directionType::fwd, 180, 70, 0.05);
    robot.makeTurn(90, true);
    //robot.intake.spin(directionType::fwd, 50, velocityUnits::pct);
    robot.goStraight(15, directionType::fwd, 90, 70, 0.05);

    robot.move(-50, -50, 50, 50);
    vex::task::sleep(700);
    robot.stop(vex::brakeType::coast);
    robot.roller.spin(vex::directionType::rev, 70, vex::velocityUnits::pct);
    vex::task::sleep(1000);
    robot.roller.stop(vex::brakeType::coast);
    //robot.intake.stop(vex::brakeType::coast);

    robot.goStraight(2 * 24, directionType::fwd, 270, 50);
    robot.makeTurn(105, true);

    robot.flywheel.spin(directionType::fwd, 50, velocityUnits::pct);
    vex::task::sleep(2500);
    robot.intake.spin(directionType::fwd, 50, velocityUnits::pct);

    vex::task::sleep(7000);
    robot.flywheel.stop(brakeType::coast);
    robot.intake.stop(brakeType::coast);
    robot.makeTurn(225, true);
    robot.goStraight(5, directionType::fwd, 315, 50);
    robot.goStraight(5, directionType::fwd, 45, 50);
    
    robot.expander.spin(vex::directionType::rev, 40, vex::velocityUnits::pct);
    vex::task::sleep(7000);
    robot.expander.stop(vex::brakeType::coast);
    return;
    

}  //autonWithXD

void testTask(XDriveRobot& robot) {
    task t2(spinForever);
    vex::task::sleep(2000);
    t2.stop();
    robot.goStraight(10, directionType::fwd, 0, 60);
}

void driveWithXD(XDriveRobot& robot, vex::controller& rc, double kp) {
    bool isShooting = false;
    int counter = 0; // for delayed intake start after flywheel

    int intakeStatus = 0;

    controller::button& btnShootStart = rc.ButtonLeft;
    controller::button& btnShootStop = rc.ButtonRight;

    controller::button& btnIntakeFwd = rc.ButtonL1;
    controller::button& btnIntakeRev = rc.ButtonL2;

    controller::button& btnRollerFwd = rc.ButtonR1;
    controller::button& btnRollerRev = rc.ButtonR2;

    controller::button& btnExpanderFwd = rc.ButtonUp;
    controller::button& btnExpanderRev = rc.ButtonDown;

    while (1) {
        //rs.print("velocity: %f", flywheel.velocity(velocityUnits::rpm));

        double xSpeed = logDrive(rc.Axis4.position(percentUnits::pct)); // run experiment to see which side is positive/negative
        double ySpeed = logDrive(rc.Axis3.position(percentUnits::pct));

        // force move along x or y direction, no striffing
        if (fabs(xSpeed) > 2.0 * fabs(ySpeed)) {
            ySpeed = 0;
        }
        else {
            if (fabs(xSpeed) < 0.5 * fabs(ySpeed)) {
                xSpeed = 0;
            }
            else {
                xSpeed = 0;
                ySpeed = 0;
            }
        }

        double spinSpeed = logDriveT(rc.Axis1.position(percentUnits::pct)*3/4);
        
        //rs.print("user speed: %.1f, %.1f, %.f", xSpeed, ySpeed, spinSpeed);

        // based on motor config: +, -, -, +
        double blspeed = xSpeed - ySpeed - spinSpeed;
        double brspeed = -xSpeed - ySpeed + spinSpeed;
        double flspeed = xSpeed + ySpeed + spinSpeed;
        double frspeed = -xSpeed + ySpeed - spinSpeed;

        robot.move(blspeed, brspeed, flspeed, frspeed);

        if (btnShootStart.pressing()){
            isShooting = true;
            robot.flywheel.spin(directionType::fwd, 90, velocityUnits::pct);
        }

        if (rc.ButtonY.pressing()) {
            isShooting = true;
            robot.flywheel.spin(directionType::fwd, 70, velocityUnits::pct);
        }
         
        if (btnShootStop.pressing()) {
            isShooting = false;
            counter = 0;
            //rs.print("STOP flywheel: %d", counter);
            robot.flywheel.stop(brakeType::coast);
            robot.intake.stop(brakeType::coast);
            intakeStatus = 0;
        }

        if (btnIntakeFwd.pressing()) {
            intakeStatus = 0;
            robot.intake.spin(directionType::fwd, 50, velocityUnits::pct);
        }
        else if (btnIntakeRev.pressing()) {
            intakeStatus = 0;
            robot.intake.spin(directionType::rev, 50, velocityUnits::pct);
        }
        else if (rc.ButtonA.pressing()) {
            intakeStatus = -1;
        }
        else if (intakeStatus == -1 && !rc.ButtonA.pressing()) {
            intakeStatus = 1;
        }
        else if (!isShooting) {
            robot.intake.stop(brakeType::coast);
            intakeStatus = 0;
        }

        if (intakeStatus == 1) {
            robot.intake.spin(directionType::fwd, 50, velocityUnits::pct);
        }
        else if (intakeStatus == -1) {
            robot.intake.stop(brakeType::coast);
        }

        if (btnRollerFwd.pressing()) {
            robot.roller.spin(directionType::fwd, 80, velocityUnits::pct);
        }
        else if (btnRollerRev.pressing()) {
            robot.roller.spin(directionType::rev, 80, velocityUnits::pct);
        }
        else {
            robot.roller.stop(brakeType::coast);
        }

        if (btnExpanderFwd.pressing()) {
            robot.expander.spin(directionType::fwd, 40, velocityUnits::pct);
        }
        else if (btnExpanderRev.pressing()) {
            robot.expander.spin(directionType::rev, 60, velocityUnits::pct);
        }
        else {
            robot.expander.stop(brakeType::coast);
        }

        if (isShooting && flywheel.velocity(velocityUnits::rpm) > 95) {
            robot.flywheel.spin(directionType::fwd, 53, velocityUnits::pct);
        }

        if (isShooting && flywheel.velocity(velocityUnits::rpm) > 95) {
            robot.intake.spin(directionType::fwd, 45, velocityUnits::pct);
            intakeStatus = 1;
        }

        if (isShooting) {
            counter++;
        }
        //rs.print("%d: %d", isShooting, counter);
        task::sleep(10);
    }
}

void turnNinety(XDriveRobot& robot) {
    //rs.print("start heading: %.1f", robot.inertialSensor.heading());
    robot.makeTurn(90, true);
    //rs.print("end heading: %.1f", robot.inertialSensor.heading());
}


void pre_auton( void ) {
/*
  if (AT_HOME == false) { 
    inertialSensor.calibrate();
    int timeCount = 0;

    while (inertialSensor.isCalibrating()) {
    timeCount+= 20;
    vex::task::sleep(20);
    }

    rs.print("pre auton calibration time: %d", timeCount);
    }
    */
}

void autonomous( void ) {
  //if (AT_HOME) { robot.calibrate(); rs.print("auton: calibrated %d", 1);}
  robot.calibrate();
  vex::task::sleep(3000);
  FarSkillsDiagonal(robot);
}


void usercontrol( void ) {
  driveWithXD(robot, rc, 0);
} // usercontrol

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //robot.calibrate();
  //rs.print("main auton: calibrated %d", 1);

  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton();

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}  // main()
