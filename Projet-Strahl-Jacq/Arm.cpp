#include "Arm.h"
#include <Math.h>
#include <Arduino.h>

#define DEBUG

Arm::Arm(int8_t pinBase, int8_t pinShoulder, int8_t pinElbow) {
#ifdef DEBUG
  Serial.println("New Arm");
#endif

  mBase->pin = pinBase;
  mShoulder->pin = pinShoulder;
  mElbow->pin = pinElbow;

  ArmInit();
}

Arm::~Arm() {
#ifdef DEBUG
  Serial.println("Delete Arm");
#endif
}

// Arm init ---------------------------------------------------- 

unsigned int Arm::ArmInit(int soft_start_level) {
#ifdef DEBUG
  Serial.println("Arm::ArmInit Start");
#endif

  //Calling Braccio.begin(SOFT_START_DISABLED) the Softstart is disabled and you can use the pin 12
  if(soft_start_level!=SOFT_START_DISABLED){
    pinMode(SOFT_START_CONTROL_PIN,OUTPUT);
    digitalWrite(SOFT_START_CONTROL_PIN,LOW);
  }
  
  AttachServos();
  InitPositionServos();
  SetRangeServos();

  if(soft_start_level!=SOFT_START_DISABLED)
        _softStart(soft_start_level);
        
#ifdef DEBUG
  Serial.println("Arm::ArmInit End");
#endif
  return 1;
}

void Arm::_softStart(int soft_start_level) {
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

void Arm::_softwarePWM(int high_time, int low_time) {
#ifdef DEBUG
  //Serial.println("Arm::_softwarePWM");
#endif

  digitalWrite(SOFT_START_CONTROL_PIN,HIGH);
  delayMicroseconds(high_time);
  digitalWrite(SOFT_START_CONTROL_PIN,LOW);
  delayMicroseconds(low_time); 
}

void Arm::AttachServos() {
#ifdef DEBUG
  Serial.println("Arm::AttachServos");
#endif

  mBase->     servo->attach(mBase->pin);
  mShoulder-> servo->attach(mShoulder->pin);
  mElbow->    servo->attach(mElbow->pin);
}

void Arm::InitPositionServos() {
#ifdef DEBUG
  Serial.println("Arm::InitPositionServos");
#endif
  
  mBase->     servo->write(BASE_ANGLE_INIT);
  delay(1000);
  mShoulder-> servo->write(SHOULDER_ANGLE_INIT);
  delay(1000);
  mElbow->    servo->write(ELBOW_ANGLE_INIT);
  delay(1000);

  mBase->     angle = BASE_ANGLE_INIT;
  mShoulder-> angle = SHOULDER_ANGLE_INIT;
  mElbow->    angle = ELBOW_ANGLE_INIT;
}

void Arm::SetRangeServos() {
#ifdef DEBUG
  Serial.println("Arm::SetRangeServos");
#endif

  mBase->     range = BASE_RANGE;
  mShoulder-> range = SHOULDER_RANGE;
  mElbow->    range = ELBOW_RANGE;
}

void Arm::CalibrateServos() {
#ifdef DEBUG
  Serial.println("Arm::CalibrateServo");
#endif

  mBase->     servo->write(90);
  delay(1000);
  mShoulder-> servo->write(90);
  delay(1000);
  mElbow->    servo->write(90);
  delay(1000);
}

// Servo movements ---------------------------------------------------- 

void Arm::MoveToTarget() {
#ifdef DEBUG
  Serial.println("Arm::MoveToTarget");
#endif 

  MoveServosToAngle((uint8_t) GetServoBaseAngle(), 
                    (uint8_t) GetServoShoulderAngle(), 
                    (uint8_t) GetServoElbowAngle());
}

void Arm::MoveToTarget(uint16_t timeDelay) {
#ifdef DEBUG
  Serial.println("Arm::MoveToTargetTime");
#endif 

  MoveServosToAngle((uint8_t) GetServoBaseAngle(), 
                    (uint8_t) GetServoShoulderAngle(), 
                    (uint8_t) GetServoElbowAngle(), 
                    timeDelay);
}

void Arm::MoveServosToAngle(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle) {
#ifdef DEBUG
  Serial.println("Arm::MoveServosToAngle");
#endif
  
  mBase->     anglePrevious = mBase->angle;
  mShoulder-> anglePrevious = mShoulder->angle;
  mElbow->    anglePrevious = mElbow->angle;

  mBase->     angleNext = mBase->angle;
  mShoulder-> angleNext = mShoulder->angle;
  mElbow->    angleNext = mElbow->angle;

  mBase->     angleTarget = baseAngle;
  mShoulder-> angleTarget = shoulderAngle;
  mElbow->    angleTarget = elbowAngle;

  mBase->servo->write(mBase->angleTarget);
  mShoulder->servo->write(mShoulder->angleTarget);
  mElbow->servo->write(mElbow->angleTarget);

  mBase->angle = mBase->angleTarget;
  mShoulder->angle = mShoulder->angleTarget;
  mElbow->angle = mElbow->angleTarget;
}

void Arm::MoveServosToAngle(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle, uint16_t timeDelay) {
#ifdef DEBUG
  Serial.println("Arm::MoveServosToAngleTime");
#endif

  uint8_t stepNumber = 8;
  uint16_t stepDelay = timeDelay/stepNumber;
  
  mBase->     anglePrevious = mBase->angle;
  mShoulder-> anglePrevious = mShoulder->angle;
  mElbow->    anglePrevious = mElbow->angle;

  mBase->     angleNext = mBase->angle;
  mShoulder-> angleNext = mShoulder->angle;
  mElbow->    angleNext = mElbow->angle;

  mBase->     angleTarget = baseAngle;
  mShoulder-> angleTarget = shoulderAngle;
  mElbow->    angleTarget = elbowAngle;

  mBase->     stepMoving = abs(mBase->anglePrevious     - mBase->angleTarget)     / stepNumber;
  mShoulder-> stepMoving = abs(mShoulder->anglePrevious - mShoulder->angleTarget) / stepNumber;
  mElbow->    stepMoving = abs(mElbow->anglePrevious    - mElbow->angleTarget)    / stepNumber;

  for(uint8_t i=0;i<stepNumber;i++)
  {
    if (mBase->angleNext > mBase->angleTarget) mBase->angleNext += mBase->stepMoving;
    if (mBase->angleNext < mBase->angleTarget) mBase->angleNext -= mBase->stepMoving;
    mBase->servo->write(mBase->angleNext);

    if (mShoulder->angleNext > mShoulder->angleTarget) mShoulder->angleNext += mShoulder->stepMoving;
    if (mShoulder->angleNext < mShoulder->angleTarget) mShoulder->angleNext -= mShoulder->stepMoving;
    mShoulder->servo->write(mShoulder->angleNext);

    if (mElbow->angleNext > mElbow->angleTarget) mElbow->angleNext += mElbow->stepMoving;
    if (mElbow->angleNext < mElbow->angleTarget) mElbow->angleNext -= mElbow->stepMoving;
    mElbow->servo->write(mElbow->angleNext);

#ifdef DEBUG
    //String outString = "ServosAngle : base = " + String(mBase->angleNext) + " shoulder = " + String(mShoulder->angleNext) +  " elbow =" +  String(mElbow->angleNext); 
    //Serial.println(outString);
    Serial.print("ServosAngle : base = ");
    Serial.print(mBase->angleNext);
    Serial.print(" shoulder = ");
    Serial.print(mShoulder->angleNext);
    Serial.print(" elbow =");
    Serial.println(mElbow->angleNext);
#endif

    delay(stepDelay);
  }

  mBase->servo->write(mBase->angleTarget);
  mShoulder->servo->write(mShoulder->angleTarget);
  mElbow->servo->write(mElbow->angleTarget);

  mBase->angle = mBase->angleTarget;
  mShoulder->angle = mShoulder->angleTarget;
  mElbow->angle = mElbow->angleTarget;
}

// Coordinates ---------------------------------------------------

void Arm::SetCoordinateTarget(float x, float y, float z) {
#ifdef DEBUG
  Serial.println("Arm::SetCoordinateTarget");
#endif

  mTarget->x         = x;
  mTarget->y         = y;
  mTarget->z         = z;

  mTarget->module    = sqrt(sq(x) + sq(y));
  mTarget->argument  = asin(y / mTarget->module);
}

void Arm::SetCoordinatePolarTarget(float module, float argument, float z) {
#ifdef DEBUG
  Serial.println("Arm::SetCoordinatePolarTarget");
#endif

  mTarget->module    = module;
  mTarget->argument  = argument;
  mTarget->z         = z;

  mTarget->x         = cos(mTarget->argument) * (mTarget->module - mBase->range);
  mTarget->y         = sin(mTarget->argument) * (mTarget->module - mBase->range);
}

// Angles ----------------------------------------------------

float Arm::ServoMaxAngle(Servomotor *servo, uint8_t angle) {
#ifdef DEBUG
  Serial.println("Arm::ServoMaxAngle");
#endif

  if(angle<servo->angleMin) return servo->angleMin;
  if(angle>servo->angleMax) return servo->angleMax;
  return angle;
}

float Arm::GetServoBaseAngle() {
#ifdef DEBUG
  Serial.println("Arm::GetServoBaseAngle");
#endif

  return MaxAngle(mTarget->argument);
}

float Arm::GetServoShoulderAngle() {
#ifdef DEBUG
  Serial.println("Arm::GetServoShoulderAngle");
#endif

  float c = sqrt((sq(mTarget->module - mBase->range) + sq(mTarget->z)));
  float error = atan(mTarget->z / (mTarget->module - mElbow->range));
  return MaxAngle(error - acos((-sq(mElbow->range) + sq(mShoulder->range) + sq(c)) / (2 * mShoulder->range * c)));
}

float Arm::GetServoElbowAngle() {
#ifdef DEBUG
  Serial.println("Arm::GetServoElbowAngle");
#endif

  float c = sqrt((sq(mTarget->module - mElbow->range) + sq(mTarget->z)));
  return MaxAngle((-PI / 2) + acos((-sq(c) + sq(mShoulder->range) + sq(mBase->range)) / (2 * mShoulder->range * mBase->range)));
}

float Arm::MaxAngle(float angle) {
#ifdef DEBUG
  Serial.println("Arm::MaxAngle");
#endif

  if (angle < (-PI / 2)) return (-PI / 2);
  if (angle > (PI / 2)) return (PI / 2);
  return angle;
}

// Getter ----------------------------------------------------

uint8_t Arm::GetBaseAngle() {
#ifdef DEBUG
  Serial.println("Arm::GetBaseAngle");
#endif

  return mBase->angle;
}

uint8_t Arm::GetShoulderAngle() {
#ifdef DEBUG
  Serial.println("Arm::GetShoulderAngle");
#endif

  return mShoulder->angle;
}

uint8_t Arm::GetElbowAngle()  {
#ifdef DEBUG
  Serial.println("Arm::GetElbowAngle");
#endif

  return mElbow->angle;
}
