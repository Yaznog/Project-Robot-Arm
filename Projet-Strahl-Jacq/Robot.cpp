#include "Robot.h"

//#define DEBUG

Robot::Robot() {
#ifdef DEBUG
  Serial.println("New Robot");
#endif

  RobotInit();

  // Create arm (pinBase, pinShoulder, pinElbow)
  mArm = new Arm(11, 10, 9);
  // Create hand (pinWrist_ver, pinWrist_rot, pinGripper)
  mHand  = new Hand(5, 6, 3);
}

Robot::~Robot() {
#ifdef DEBUG
  Serial.println("Robot deleted");
#endif

  mArm->~Arm();
  mHand->~Hand();
}

// Robot init ---------------------------------------------------- 

unsigned int Robot::RobotInit(int soft_start_level) {
#ifdef DEBUG
  Serial.println("Robot::RobotInit Start");
#endif

  //Calling Braccio.begin(SOFT_START_DISABLED) the Softstart is disabled and you can use the pin 12
  if(soft_start_level!=SOFT_START_DISABLED){
    pinMode(SOFT_START_CONTROL_PIN,OUTPUT);
    digitalWrite(SOFT_START_CONTROL_PIN,LOW);
  }

  if(soft_start_level!=SOFT_START_DISABLED)
        _softStart(soft_start_level);
        
#ifdef DEBUG
  Serial.println("Arm::RobotInit End");
#endif
  return 1;
}

void Robot::_softStart(int soft_start_level) {
#ifdef DEBUG
  Serial.println("Arm::_softStart");
#endif

  long int tmp=millis();
  while(millis()-tmp < LOW_LIMIT_TIMEOUT)
    _softwarePWM(80+soft_start_level, 450 - soft_start_level);   //the sum should be 530usec  

  while(millis()-tmp < HIGH_LIMIT_TIMEOUT)
    _softwarePWM(75 + soft_start_level, 430 - soft_start_level); //the sum should be 505usec

  digitalWrite(SOFT_START_CONTROL_PIN,HIGH);
}

void Robot::_softwarePWM(int high_time, int low_time) {
#ifdef DEBUG
  //Serial.println("Arm::_softwarePWM");
#endif

  digitalWrite(SOFT_START_CONTROL_PIN,HIGH);
  delayMicroseconds(high_time);
  digitalWrite(SOFT_START_CONTROL_PIN,LOW);
  delayMicroseconds(low_time); 
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
  //mHand->InitPositionServos();
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
