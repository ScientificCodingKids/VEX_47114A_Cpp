#include "vex.h"
#include <iostream>
#include <cmath>
#include <cassert>
#include "nutils.hpp"
#include "motion.hpp"

using namespace vex;
using namespace std;

competition Competition;


class Roboto: public DriveTrainBase {
  public:
    Roboto(vex::motor& bl, vex::motor& br, vex::motor& fl, vex::motor& fr, 
      vex::brain& brn, vex::inertial& ins, 
      vex::motor& aIntake): DriveTrainBase(bl, br, fl, fr, brn, ins), intake(aIntake) {
      // TODO
    }
  
    vex::motor& intake;
};  // class Roboto



RollingScreen rs(Brain.Screen);


void pre_auton(Roboto& robot) {
  rs.print("Enter pre_auton(): %d ", 0);

  //robot.calibrate();

}

void autonomous( void ) {

}


void usercontrol( void ) {
  // remote control key mapping
  // https://kb.vex.com/hc/en-us/articles/360035954651-Using-Blocks-for-Controller-Buttons-Joysticks-in-VEXcode-V5

  bool enableDrivetrain = false;
  bool enableIntake = true;

  //bool isRunning = false;
  int currSpeed = 20;
  int maxSpeed = 90;

  bool isMotorFLOn = false;
  bool isMotorFROn = false;
  bool isMotorBLOn = false;
  bool isMotorBROn = false;


  vex::directionType intakeRotDir = vex::directionType::fwd;
  rs.print("START ... \n");

  while (1) {
    if (rc.ButtonR1.pressing()) {
      currSpeed = min(currSpeed + 10, maxSpeed);
    }

    if (rc.ButtonR2.pressing()) {
      currSpeed = max(currSpeed - 10, 0);
    }

    if (enableDrivetrain) {
      if (rc.ButtonL2.pressing()) {
        isMotorFLOn = false;
        isMotorFROn = false;
        isMotorBLOn = false;
        isMotorBROn = false;
      }

      if (rc.ButtonY.pressing()) {
        isMotorFLOn = true;
      }

      if (rc.ButtonX.pressing()) {
        isMotorFROn = true;
      }

      if (rc.ButtonB.pressing()) {
        isMotorBLOn = true;
      }

      if (rc.ButtonA.pressing()) {
        isMotorBROn = true;
      }

      if (isMotorFLOn) {
        frontleftdrive.setVelocity(currSpeed, vex::velocityUnits::pct);
        frontleftdrive.spin(vex::directionType::fwd);
      }
      if (isMotorFROn) {
        frontrightdrive.setVelocity(currSpeed, vex::velocityUnits::pct);
        frontrightdrive.spin(vex::directionType::fwd);
      }

      if (isMotorBLOn) {
        backleftdrive.setVelocity(currSpeed, vex::velocityUnits::pct);
        backleftdrive.spin(vex::directionType::fwd);
      }

      if (isMotorBROn) {
        backrightdrive.setVelocity(currSpeed, vex::velocityUnits::pct);
        backrightdrive.spin(vex::directionType::fwd);
      }
          
    }  // if enableDrivetrain

    if (enableIntake) {
      if (rc.ButtonUp.pressing()) {
        intakeRotDir = vex::directionType::fwd;
      }

      if (rc.ButtonDown.pressing()) {
        intakeRotDir = vex::directionType::rev;
      }

      intake.setVelocity(currSpeed, velocityUnits::pct);

      rs.print("Intake SPIN at %.1f \n", currSpeed);

      intake.spin(intakeRotDir);


    }  // if enableIntake
      

    // if (isRunning) {
    //   rs.print("%.0f | %.0f => v:%.0f, i:%.0f, p:%.0f \n", currSpeed, cat.velocity(), cat.voltage(), cat.current(), cat.power());
    // }
    
    task::sleep(100);
  }  // while
} // usercontrol




int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // our own code ONLY AFTER vexcodeInit()
  Roboto robot(backleftdrive, backrightdrive, frontleftdrive, frontrightdrive, Brain, inertialSensor, intake);
  robot.setRollingScreen(&rs);

  Competition.autonomous( autonomous );
  Competition.drivercontrol( usercontrol );

  pre_auton(robot);

  while(1){
    vex::task::sleep(100);
  }
  return 0;
}