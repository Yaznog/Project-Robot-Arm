#include "Robot.h"

#define DEBUG

Robot::Robot() {
#ifdef DEBUG
  Serial.println("New Robot");
#endif

  // Create arm (pinBase, pinShoulder, pinElbow)
  mArm = new Arm(11, 10, 9);
  // Create hand (pinWrist_ver, pinWrist_rot, pinGripper)
  //mHand  = new Hand(5, 6, 3);
}

Robot::~Robot() {
#ifdef DEBUG
  Serial.println("Robot deleted");
#endif

  //mHand->~Hand();
  mArm->~Arm();
}

// Servo Movements Arm ---------------------------------------------------- 

void Robot::MoveWristToCoordinate(float x, float y, float z) {
#ifdef DEBUG
  Serial.println("Robot::MoveWristToCoordinate");
#endif
  mArm->SetCoordinateTarget(x, y, z);
  mArm->MoveToTarget();
}

void Robot::MoveWristToCoordinate(float x, float y, float z, uint16_t timeDelay) {
#ifdef DEBUG
  Serial.println("Robot::MoveWristToCoordinateTime");
#endif
  mArm->SetCoordinateTarget(x, y, z);
  mArm->MoveToTarget(timeDelay);
}

void Robot::MoveWristToCoordinatePolar(float module, float argument, float z) {
#ifdef DEBUG
  Serial.println("Robot::MoveWristToCoordinatePolar");
#endif
  mArm->SetCoordinatePolarTarget(module, argument, z);
  mArm->MoveToTarget();
}

void Robot::MoveWristToCoordinatePolar(float module, float argument, float z, uint16_t timeDelay) {
#ifdef DEBUG
  Serial.println("Robot::MoveWristToCoordinatePolarTime");
#endif
  mArm->SetCoordinatePolarTarget(module, argument, z);
  mArm->MoveToTarget(timeDelay);
}

// Calibrate ---------------------------------------------------- 

void Robot::CalibrateArm() {
#ifdef DEBUG
  Serial.println("Robot::CalibrateArm");
#endif 
  mArm->CalibrateServos();
}

void Robot::CalibrateHand() {
#ifdef DEBUG
  Serial.println("Robot::CalibrateArm");
#endif 
  //mHand->CalibrateServos();
}
