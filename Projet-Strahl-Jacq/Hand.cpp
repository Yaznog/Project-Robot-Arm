#include "Hand.h"
#include <Math.h>
#include <Arduino.h>

//#define DEBUG

Hand::Hand(int8_t pinWristVer, int8_t pinWristRot, int8_t pinGripper) {
#ifdef DEBUG
  Serial.println("New Hand");
#endif

  mWristVer-> pin = pinWristVer;
  mWristRot-> pin = pinWristRot;
  mGripper->  pin = pinGripper;

  HandInit();
}

Hand::~Hand() {
#ifdef DEBUG
  Serial.println("Delete Hand");
#endif
}

// Hand init ---------------------------------------------------- 

void Hand::HandInit() {
#ifdef DEBUG
  Serial.println("Hand::HandInit Start");
#endif

  AttachServos();
  InitPositionServos();
  SetRangeServos();
        
#ifdef DEBUG
  Serial.println("Hand::HandInit End");
#endif
}

void Hand::AttachServos() {
#ifdef DEBUG
  Serial.println("Hand::AttachServos");
#endif

  mWristVer-> servo->attach(mWristVer-> pin);
  mWristRot-> servo->attach(mWristRot-> pin);
  mGripper->  servo->attach(mGripper->  pin);
}

void Hand::InitPositionServos() {
#ifdef DEBUG
  Serial.println("Hand::InitPositionServos");
#endif

  //MoveServosToAngle(WRISTVER_ANGLE_INIT, WRISTROT_ANGLE_INIT, GRIPPER_ANGLE_INIT);
}

void Hand::SetRangeServos() {
#ifdef DEBUG
  Serial.println("Hand::SetRangeServo");
#endif

  mWristVer-> range = WRISTVER_RANGE;
  mWristRot-> range = WRISTROT_RANGE;
  mGripper->  range = GRIPPER_RANGE;
}

void Hand::CalibrateServos() {
#ifdef DEBUG
  Serial.println("Hand::CalibrateServo");
#endif

  //MoveServosToAngle(90, 90, 90);
}

// Servo movements ---------------------------------------------------- 
/*
void Hand::MoveToTarget() {
#ifdef DEBUG
  Serial.println("Hand::MoveToTarget");
#endif 

  MoveServosToAngle((uint8_t) GetServoBaseAngle(), 
                    (uint8_t) GetServoShoulderAngle(), 
                    (uint8_t) GetServoAngle());
}

void Hand::MoveToTarget(uint16_t timeDelay) {
#ifdef DEBUG
  Serial.println("Hand::MoveToTargetTime");
#endif 

  MoveServosToAngle((uint8_t) GetServoBaseAngle(), 
                    (uint8_t) GetServoShoulderAngle(), 
                    (uint8_t) GetServoElbowAngle(), 
                    timeDelay);
}

void Hand::MoveServosToAngle(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle) {
#ifdef DEBUG
  Serial.println("Hand::MoveServosToAngle");
#endif

  mWristVer->     angle = baseAngle;
  mWristRot-> angle = shoulderAngle;
  mGripper->    angle = elbowAngle;
  
  mWristVer->     servo->write(mWristVer->     angle);
  mWristRot-> servo->write(mWristRot-> angle);
  mGripper->    servo->write(mGripper->    angle);

#ifdef DEBUG
  Serial.print("     ServosAngle : base = ");
  Serial.print(mWristVer->    angle);
  Serial.print(" shoulder = ");
  Serial.print(mWristRot->angle);
  Serial.print(" elbow = ");
  Serial.println(mGripper-> angle);
#endif
}

void Hand::MoveServosToAngle(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle, uint16_t timeDelay) {
#ifdef DEBUG
  Serial.println("Hand::MoveServosToAngleTime");
#endif

  uint8_t stepNumber = 5;
  uint16_t stepDelay = timeDelay/stepNumber;
  
#ifdef DEBUG
  Serial.print("     stepDelay = ");
  Serial.println(stepDelay);
#endif

#ifdef DEBUG
  Serial.print("     Angle : base = ");
  Serial.print(baseAngle);
  Serial.print(" shoulder = ");
  Serial.print(shoulderAngle);
  Serial.print(" elbow = ");
  Serial.println(elbowAngle);
#endif

#ifdef DEBUG
  Serial.print("     ServosAngle : base = ");
  Serial.print(mWristVer->    angle);
  Serial.print(" shoulder = ");
  Serial.print(mWristRot->angle);
  Serial.print(" elbow = ");
  Serial.println(mGripper-> angle);
#endif

  mWristVer->     stepMoving = (baseAngle     - mWristVer->     angle) / stepNumber;
  mWristRot-> stepMoving = (shoulderAngle - mWristRot-> angle) / stepNumber;
  mGripper->    stepMoving = (elbowAngle    - mGripper->    angle) / stepNumber;

#ifdef DEBUG
    Serial.print("     StepMoving : base = ");
    Serial.print(mWristVer->    stepMoving);
    Serial.print(" shoulder = ");
    Serial.print(mWristRot->stepMoving);
    Serial.print(" elbow = ");
    Serial.println(mGripper-> stepMoving);
#endif
  
  for(uint8_t i=1;i<stepNumber;i++)
  {
    mWristVer->     angle += mWristVer->     stepMoving;
    mWristRot-> angle += mWristRot-> stepMoving;
    mGripper->    angle += mGripper->    stepMoving;
    
    mWristVer->     servo->write(mWristVer->     angle);
    mWristRot-> servo->write(mWristRot-> angle);
    mGripper->    servo->write(mGripper->    angle);

#ifdef DEBUG
    Serial.print("     ServosAngle : base = ");
    Serial.print(mWristVer->angle);
    Serial.print(" shoulder = ");
    Serial.print(mWristRot->angle);
    Serial.print(" elbow = ");
    Serial.println(mGripper->angle);
#endif

    delay(stepDelay);
  }

  mWristVer->     angle = baseAngle;
  mWristRot-> angle = shoulderAngle;
  mGripper->    angle = elbowAngle;
  
  mWristVer->     servo->write(mWristVer->     angle);
  mWristRot-> servo->write(mWristRot-> angle);
  mGripper->    servo->write(mGripper->    angle);

#ifdef DEBUG
    Serial.print("     EndMovement : base = ");
    Serial.print(mWristVer->angle);
    Serial.print(" shoulder = ");
    Serial.print(mWristRot->angle);
    Serial.print(" elbow = ");
    Serial.println(mGripper->angle);
#endif
}*/

// Coordinates ---------------------------------------------------
/*
void Hand::SetCoordinateTarget(float x, float y, float z) {
#ifdef DEBUG
  Serial.println("Hand::SetCoordinateTarget");
#endif

  mTarget->x         = x;
  mTarget->y         = y;
  mTarget->z         = z;

  mTarget->module    = sqrt(sq(x) + sq(y));
  mTarget->argument  = asin(y / mTarget->module);

  
}

void Hand::SetCoordinatePolarTarget(float module, float argument, float z) {
#ifdef DEBUG
  Serial.println("Hand::SetCoordinatePolarTarget");
#endif

  mTarget->module    = module;
  mTarget->argument  = argument;
  mTarget->z         = z;

  mTarget->x         = cos(mTarget->argument) * (mTarget->module - mWristVer->range);
  mTarget->y         = sin(mTarget->argument) * (mTarget->module - mWristVer->range);
}*/

// Angles ----------------------------------------------------
/*
float Hand::ServoMaxAngle(Servomotor *servo, uint8_t angle) {
#ifdef DEBUG
  Serial.println("Hand::ServoMaxAngle");
#endif

  if(angle<servo->angleMin) return servo->angleMin;
  if(angle>servo->angleMax) return servo->angleMax;
  return angle;
}

float Hand::GetServoBaseAngle() {
#ifdef DEBUG
  Serial.println("Hand::GetServoBaseAngle");
#endif
  bool MODE = 1;
  float angle;

  float arg = (float)mTarget->argument;
  
  if(MODE) angle = (arg + PI/2.0) * 180.0/PI;
  else angle = -(arg - PI/2.0) * 180.0/PI;

#ifdef DEBUG
  Serial.print("     ");
  Serial.println(angle);
#endif

  return angle;
}

float Hand::GetServoShoulderAngle() {
#ifdef DEBUG
  Serial.println("Hand::GetServoShoulderAngle");
#endif
  bool MODE = 1;
  float angle;

  float a = (float)mGripper->range;
  float b = (float)sqrt(sq(mTarget->module) + sq(mTarget->z));
  float c = (float)mWristRot->range;

  float mod = (float)mTarget->module;

  if(MODE) angle = (acos(mod/b)+acos( (sq(a) - sq(b) - sq(c))/(2*b*c))) * 180.0/PI;
  else angle = -(acos(mod/b)+acos( (sq(a) - sq(b) - sq(c))/(2*b*c))-PI) * 180.0/PI;

#ifdef DEBUG
  Serial.print("     ");
  Serial.println(angle);
#endif

  return angle;
}

float Hand::GetServoElbowAngle() {
#ifdef DEBUG
  Serial.println("Hand::GetServoElbowAngle");
#endif
  bool MODE = 1;
  float angle;

  float a = (float)mGripper->range;
  float b = (float)sqrt(sq(mTarget->module) + sq(mTarget->z));
  float c = (float)mWristRot->range;

  float mod = (float)mTarget->module;

  if(MODE) angle = -( acos( (sq(b) - sq(a) - sq(c)) / (-2*a*c) ) - PI ) * 180.0/PI;
  else angle = ( acos( (sq(b) - sq(a) - sq(c)) / (-2*a*c) ) ) * 180.0/PI;

#ifdef DEBUG
  Serial.print("     ");
  Serial.println(angle);
#endif

  return angle;
}*/

float Hand::MaxAngle(float angle) {
#ifdef DEBUG
  Serial.println("Hand::MaxAngle");
#endif

  if (angle < (-PI / 2)) return (-PI / 2);
  if (angle > (PI / 2)) return (PI / 2);
  return angle;
}

float Hand::RadToDegree(float angle) {
#ifdef DEBUG
  Serial.println("Hand::RadToDegree");
#endif
  return (180.0/PI)*angle;
}

// Getter ----------------------------------------------------

uint8_t Hand::GetWristVerAngle() {
#ifdef DEBUG
  Serial.println("Hand::GetWristVerAngle");
#endif

  return mWristVer->angle;
}

uint8_t Hand::GetWristRotAngle() {
#ifdef DEBUG
  Serial.println("Hand::GetWristRotAngle");
#endif

  return mWristRot->angle;
}

uint8_t Hand::GetGripperAngle()  {
#ifdef DEBUG
  Serial.println("Hand::GetGripperAngle");
#endif

  return mGripper->angle;
}
