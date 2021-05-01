
#include "v5.h"
#include "v5_vcs.h"

class DriveBase {
private:
    vex::motor& _backleftdrive;
    vex::motor& _backrightdrive;
    vex::motor& _frontleftdrive;
    vex::motor& _frontrightdrive;
    vex::inertial& _inertialSensor; 
    ScrollingScreen<int>& _ss;  // reference cannot be assigned, only to be initialized
public:
    DriveBase(vex::motor& backleftdrive, vex::motor& backrightdrive, vex::motor& frontleftdrive, \
      vex::motor& frontrightdrive, ScrollingScreen<int>& ss, vex::inertial& _inertialSensor);
    void resetDriveTrainRotation();
    void stopDriveTrain();
    void makeTurn(double baseSpeed, double minStartSpeed, double minEndSpeed, bool turnLeft);
    void goStraight(double rotationsToGo, double baseSpeed, double minStartSpeed, double minEndSpeed, double kp, bool goReverse=false, bool useGyro=false);
    // void goStraightWithGyro(double rotationsToGo, double baseSpeed, double minStartSpeed, double minEndSpeed, double kp);
}; // class Drivebase

DriveBase::DriveBase(vex::motor& _backleftdrive, vex::motor& _backrightdrive, 
    vex::motor& _frontleftdrive, vex::motor& _frontrightdrive,
    ScrollingScreen<int>& ss, vex::inertial& _inertialSensor):
    _backleftdrive(backleftdrive), _backrightdrive(backrightdrive),
    _frontleftdrive(frontleftdrive), _frontrightdrive(frontrightdrive),
    _ss(ss), _inertialSensor(inertialSensor) {
}

void DriveBase::resetDriveTrainRotation() {
  this->_backleftdrive.resetPosition();
  this->_frontleftdrive.resetPosition();

  this->_backrightdrive.resetPosition();
  this->_frontrightdrive.resetPosition();

  this->_inertialSensor.calibrate();
}

void DriveBase::stopDriveTrain() {
  this->_backleftdrive.stop();
  this->_frontleftdrive.stop();

  this->_backrightdrive.stop();
  this->_frontrightdrive.stop();

  this->_inertialSensor.calibrate();
}

void DriveBase::makeTurn(double baseSpeed, double minStartSpeed, double minEndSpeed, bool turnLeft) {
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
  using namespace vex;
  _inertialSensor.calibrate();

  bool is_done = false;

  while (_inertialSensor.isCalibrating()) {
    vex::wait(50, timeUnits::msec);
  }

  _inertialSensor.resetRotation();

  resetDriveTrainRotation();

  double speed = 0.0;

  this->_ss.print("Initial: %.1f", _inertialSensor.rotation());

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

    double r = turnLeft ? (360 - _inertialSensor.rotation()): _inertialSensor.rotation(); 

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
    this->_backleftdrive.setVelocity(speed, vex::percentUnits::pct);
    this->_frontleftdrive.setVelocity(speed, vex::percentUnits::pct);

    this->_backrightdrive.setVelocity(speed, vex::percentUnits::pct);
    this->_frontrightdrive.setVelocity(speed, vex::percentUnits::pct);

    this->_backleftdrive.spin(diLeftWheels);
    this->_frontleftdrive.spin(diLeftWheels);
    
    this->_backrightdrive.spin(diRightWheels);
    this->_frontrightdrive.spin(diRightWheels);

    this->_ss.print("Rotate %.1f , %.1f, %s", r, speed, is_done? "true": "false");
    vex::task::sleep(10);
  } // while

  // FINALE
  stopDriveTrain();

  this->_ss.print("makeTurn() ENDS");
}


void DriveBase::goStraight(double rotationsToGo, double baseSpeed, double minStartSpeed, double minEndSpeed, double kp, bool goReverse, bool useGyro) {
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

  using namespace vex;
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
  while (std::abs(this->_backleftdrive.rotation(vex::rotationUnits::deg)) < rotationsToGo * 360) {
    // set baseline speed
    double speed = baseSpeed;
    double leftRot = std::abs(this->_backleftdrive.rotation(vex::rotationUnits::deg));
    double rightRot = std::abs(this->_backrightdrive.rotation(vex::rotationUnits::deg));

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
    this->_backleftdrive.setVelocity(speed - speed * err * kp, vex::percentUnits::pct);
    this->_frontleftdrive.setVelocity(speed - speed * err * kp, vex::percentUnits::pct);

    this->_backrightdrive.setVelocity(speed + speed * err * kp, vex::percentUnits::pct);
    this->_frontrightdrive.setVelocity(speed + speed * err * kp, vex::percentUnits::pct);

    directionType di = goReverse ? directionType::rev : directionType::fwd;

    this->_backleftdrive.spin(di);
    this->_frontleftdrive.spin(di);
    
    this->_backrightdrive.spin(di);
    this->_frontrightdrive.spin(di);
    this->_ss.print("Rot: %.1f (%.1f)/ %.1f; Spd: %.1f; adj %.1f, err: %.1f", leftRot, backleftdrive.rotation(rotationUnits::deg), rightRot, speed, speed * err * kp, err);

    // GyroAdjust
    if (useGyro) {
      GyroAdjust = inertialSensor.rotation();
    }
    else {
      GyroAdjust = 0;
    }

    if (std::abs(GyroAdjust) > 0.5) {
      this->_backrightdrive.setVelocity(speed + 1/2 * GyroAdjust, vex::percentUnits::pct);
      this->_frontrightdrive.setVelocity(speed + 1/2 * GyroAdjust, vex::percentUnits::pct);

      this->_backleftdrive.setVelocity(speed - 1/2 * GyroAdjust, vex::percentUnits::pct);
      this->_frontleftdrive.setVelocity(speed - 1/2 * GyroAdjust, vex::percentUnits::pct);

      this->_backrightdrive.spin(di);
      this->_backleftdrive.spin(di);
      this->_frontrightdrive.spin(di);
      this->_frontleftdrive.spin(di);
    }

    vex::task::sleep(50);
  }

  // FINALE
  stopDriveTrain();
  this->_ss.print("goStraight() ENDS");

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
