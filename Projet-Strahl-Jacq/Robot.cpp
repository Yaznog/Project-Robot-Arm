#include "Robot.h"

//#define DEBUG

Robot::Robot() {
#ifdef DEBUG
  Serial.println("New Robot");
#endif

  RobotInit();
}

Robot::~Robot() {
#ifdef DEBUG
  Serial.println("Robot deleted");
#endif

  mArm->~Arm();
  mHand->~Hand();
}

// Robot init ---------------------------------------------------- 

void Robot::RobotInit() {
#ifdef DEBUG
  Serial.println("Robot::RobotInit Start");
#endif

  // Create arm (pinBase, pinShoulder, pinElbow)
  mArm = new Arm();
  // Create hand (pinWrist_ver, pinWrist_rot, pinGripper)
  mHand  = new Hand();
  
#ifdef DEBUG
  Serial.println("Arm::RobotInit End");
#endif
}

// Servo Movements Arm ---------------------------------------------------- 

void Robot::MoveOneServoToAngleArm(uint8_t servoID, uint8_t angle) {
#ifdef DEBUG
  Serial.println("Robot::MoveOneServoToAngleArm");
#endif
  mArm->MoveOneServoToAngle(servoID, angle);
}

void Robot::MoveOneServoToAngleArm(uint8_t servoID, uint8_t angle, uint16_t timeDelay) {
#ifdef DEBUG
  Serial.println("Robot::MoveOneServoToAngleArm");
#endif
  mArm->MoveOneServoToAngle(servoID, angle, timeDelay);
}

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

void Robot::MoveHandToAngle(uint8_t wristVerAngle, uint8_t wristRotAngle, uint8_t gripperAngle) {
#ifdef DEBUG
  Serial.println("Robot::MoveHandToAngle");
#endif

  mHand->MoveServosToAngle(wristVerAngle, wristRotAngle, gripperAngle);
}

// InitPosition ---------------------------------------------------- 

void Robot::InitPositionArm() {
#ifdef DEBUG
  Serial.println("Robot::InitPositionArm");
#endif 
  mArm->InitPositionServos();
}

void Robot::InitPositionHand() {
#ifdef DEBUG
  Serial.println("Robot::InitPositionHand");
#endif 
  mHand->InitPositionServos();
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
  mHand->CalibrateServos();
}
