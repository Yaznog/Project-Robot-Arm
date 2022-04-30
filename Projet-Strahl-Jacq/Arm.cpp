#include "Arm.h"
#include <Math.h>
#include <Arduino.h>

//#define DEBUG

Arm::Arm(int8_t pinBase, int8_t pinShoulder, int8_t pinElbow) {
#ifdef DEBUG
  Serial.println("New Arm");
#endif

  mBase->     pin = pinBase;
  mShoulder-> pin = pinShoulder;
  mElbow->    pin = pinElbow;

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

  mBase->     servo->attach(mBase->     pin);
  mShoulder-> servo->attach(mShoulder-> pin);
  mElbow->    servo->attach(mElbow->    pin);
}

void Arm::InitPositionServos() {
#ifdef DEBUG
  Serial.println("Arm::InitPositionServos");
#endif

  MoveServosToAngle(BASE_ANGLE_INIT, SHOULDER_ANGLE_INIT, ELBOW_ANGLE_INIT);
}

void Arm::SetRangeServos() {
#ifdef DEBUG
  Serial.println("Arm::SetRangeServo");
#endif

  mBase->     range = BASE_RANGE;
  mShoulder-> range = SHOULDER_RANGE;
  mElbow->    range = ELBOW_RANGE;
}

void Arm::CalibrateServos() {
#ifdef DEBUG
  Serial.println("Arm::CalibrateServo");
#endif

  MoveServosToAngle(90, 90, 90);
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

  mBase->     angle = baseAngle;
  mShoulder-> angle = shoulderAngle;
  mElbow->    angle = elbowAngle;
  
  mBase->     servo->write(mBase->     angle);
  mShoulder-> servo->write(mShoulder-> angle);
  mElbow->    servo->write(mElbow->    angle);

#ifdef DEBUG
  Serial.print("     ServosAngle : base = ");
  Serial.print(mBase->    angle);
  Serial.print(" shoulder = ");
  Serial.print(mShoulder->angle);
  Serial.print(" elbow = ");
  Serial.println(mElbow-> angle);
#endif
}

void Arm::MoveServosToAngle(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle, uint16_t timeDelay) {
#ifdef DEBUG
  Serial.println("Arm::MoveServosToAngleTime");
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
  Serial.print(mBase->    angle);
  Serial.print(" shoulder = ");
  Serial.print(mShoulder->angle);
  Serial.print(" elbow = ");
  Serial.println(mElbow-> angle);
#endif

  mBase->     stepMoving = (baseAngle     - mBase->     angle) / stepNumber;
  mShoulder-> stepMoving = (shoulderAngle - mShoulder-> angle) / stepNumber;
  mElbow->    stepMoving = (elbowAngle    - mElbow->    angle) / stepNumber;

#ifdef DEBUG
    Serial.print("     StepMoving : base = ");
    Serial.print(mBase->    stepMoving);
    Serial.print(" shoulder = ");
    Serial.print(mShoulder->stepMoving);
    Serial.print(" elbow = ");
    Serial.println(mElbow-> stepMoving);
#endif
  
  for(uint8_t i=1;i<stepNumber;i++)
  {
    mBase->     angle += mBase->     stepMoving;
    mShoulder-> angle += mShoulder-> stepMoving;
    mElbow->    angle += mElbow->    stepMoving;
    
    mBase->     servo->write(mBase->     angle);
    mShoulder-> servo->write(mShoulder-> angle);
    mElbow->    servo->write(mElbow->    angle);

#ifdef DEBUG
    Serial.print("     ServosAngle : base = ");
    Serial.print(mBase->angle);
    Serial.print(" shoulder = ");
    Serial.print(mShoulder->angle);
    Serial.print(" elbow = ");
    Serial.println(mElbow->angle);
#endif

    delay(stepDelay);
  }

  mBase->     angle = baseAngle;
  mShoulder-> angle = shoulderAngle;
  mElbow->    angle = elbowAngle;
  
  mBase->     servo->write(mBase->     angle);
  mShoulder-> servo->write(mShoulder-> angle);
  mElbow->    servo->write(mElbow->    angle);

#ifdef DEBUG
    Serial.print("     EndMovement : base = ");
    Serial.print(mBase->angle);
    Serial.print(" shoulder = ");
    Serial.print(mShoulder->angle);
    Serial.print(" elbow = ");
    Serial.println(mElbow->angle);
#endif
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
  bool MODE = 1;
  float angle;

  float arg = (float)mTarget->argument;
  
  if(MODE) angle = (arg + PI/2) * 180/PI;
  else angle = -(arg - PI/2) * 180/PI;

#ifdef DEBUG
  Serial.print("     ");
  Serial.println(angle);
#endif

  return angle;
}

float Arm::GetServoShoulderAngle() {
#ifdef DEBUG
  Serial.println("Arm::GetServoShoulderAngle");
#endif
  bool MODE = 1;
  float angle;

  float a = (float)mElbow->range;
  float b = (float)sqrt(sq(mTarget->module) + sq(mTarget->z));
  float c = (float)mShoulder->range;

  float mod = (float)mTarget->module;

  if(MODE) angle = (acos(mod/b)+acos( (sq(a) - sq(b) - sq(c))/(2*b*c))) * 180.0/PI;
  else angle = -(acos(mod/b)+acos( (sq(a) - sq(b) - sq(c))/(2*b*c))-PI) * 180.0/PI;

#ifdef DEBUG
  Serial.print("     ");
  Serial.println(angle);
#endif

  return angle;
}

float Arm::GetServoElbowAngle() {
#ifdef DEBUG
  Serial.println("Arm::GetServoElbowAngle");
#endif
  bool MODE = 1;
  float angle;

  float a = (float)mElbow->range;
  float b = (float)sqrt(sq(mTarget->module) + sq(mTarget->z));
  float c = (float)mShoulder->range;

  float mod = (float)mTarget->module;

  if(MODE) angle = -( acos( (sq(b) - sq(a) - sq(c)) / (-2*a*c) ) - PI ) * 180/PI;
  else angle = ( acos( (sq(b) - sq(a) - sq(c)) / (-2*a*c) ) ) * 180/PI;

#ifdef DEBUG
  Serial.print("     ");
  Serial.println(angle);
#endif

  return angle;
}

float Arm::MaxAngle(float angle) {
#ifdef DEBUG
  Serial.println("Arm::MaxAngle");
#endif

  if (angle < (-PI / 2)) return (-PI / 2);
  if (angle > (PI / 2)) return (PI / 2);
  return angle;
}

float Arm::RadToDegree(float angle) {
#ifdef DEBUG
  Serial.println("Arm::RadToDegree");
#endif
  return (180.0/PI)*angle;
}

// Getter ----------------------------------------------------

uint8_t Arm::GetBaseAngle() {
#ifdef DEBUG
  Serial.println("Arm::GetBaseAngle");
#endif

  return mBase->servo->read();
}

uint8_t Arm::GetShoulderAngle() {
#ifdef DEBUG
  Serial.println("Arm::GetShoulderAngle");
#endif

  return mShoulder->servo->read();
}

uint8_t Arm::GetElbowAngle()  {
#ifdef DEBUG
  Serial.println("Arm::GetElbowAngle");
#endif

  return mElbow->servo->read();
}
