#include "Arm.h"
#include <Math.h>
#include <Arduino.h>

#define DEBUG


Arm::Arm(int8_t pinBase, int8_t pinShoulder, int8_t pinElbow) 
{
#ifdef DEBUG
  Serial.println("New Arm");
#endif

  mBase->pin = pinBase;
  mShoulder->pin = pinShoulder;
  mElbow->pin = pinElbow;

  ArmInit();
}

Arm::~Arm() 
{
#ifdef DEBUG
  Serial.println("Delete Arm");
#endif
}

unsigned int Arm::ArmInit(int soft_start_level)
{
#ifdef DEBUG
  Serial.println("Arm::ArmInit");
#endif

  //Calling Braccio.begin(SOFT_START_DISABLED) the Softstart is disabled and you can use the pin 12
  if(soft_start_level!=SOFT_START_DISABLED){
    pinMode(SOFT_START_CONTROL_PIN,OUTPUT);
    digitalWrite(SOFT_START_CONTROL_PIN,LOW);
  }
  AttachServos();
  InitPositionServos();

  if(soft_start_level!=SOFT_START_DISABLED)
        _softStart(soft_start_level);
  return 1;
}

void Arm::_softStart(int soft_start_level)//From Braccio library
{
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

void Arm::_softwarePWM(int high_time, int low_time)//From Braccio library
{
#ifdef DEBUG
  Serial.println("Arm::_softwarePWM");
#endif

  digitalWrite(SOFT_START_CONTROL_PIN,HIGH);
  delayMicroseconds(high_time);
  digitalWrite(SOFT_START_CONTROL_PIN,LOW);
  delayMicroseconds(low_time); 
}

void Arm::AttachServos()
{
#ifdef DEBUG
  Serial.println("Arm::AttachServo");
#endif

  mBase->servo->attach(mBase->pin);
  mShoulder->servo->attach(mShoulder->pin);
  mElbow->servo->attach(mElbow->pin);
}

void Arm::InitPositionServos()
{
#ifdef DEBUG
  Serial.println("Arm::InitPositionServos");
#endif
  
  mBase->servo->write(BASE_ANGLE_INIT);
  delay(1000);
  mShoulder->servo->write(SHOULDER_ANGLE_INIT);
  delay(1000);
  mElbow->servo->write(ELBOW_ANGLE_INIT);
  delay(1000);

  mBase->angle = mBase->BASE_ANGLE_INIT;
  mShoulder->angle = mShoulder->SHOULDER_ANGLE_INIT;
  mElbow->angle = mElbow->ELBOW_ANGLE_INIT;
}

void Arm::CalibrateServos()
{
#ifdef DEBUG
  Serial.println("Arm::CalibrateServo");
#endif

  mBase->servo->write(90);
  delay(1000);
  mShoulder->servo->write(90);
  delay(1000);
  mElbow->servo->write(90);
  delay(1000);
}

void Arm::MoveServosToAngleTime(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle, uint16_t timeDelay)
{
#ifdef DEBUG
  Serial.println("Arm::MoveServosToAngleTime");
#endif

  uint8_t stepNumber = 16;
  uint16_t stepDelay = timeDelay/stepNumber;
  
  mBase->anglePrevious = mBase->angle;
  mShoulder->anglePrevious = mShoulder->angle;
  mElbow->anglePrevious = mElbow->angle;

  mBase->angleNext = mBase->angle;
  mShoulder->angleNext = mShoulder->angle;
  mElbow->angleNext = mElbow->angle;

  mBase->angleTarget = baseAngle;
  mShoulder->angleTarget = shoulderAngle;
  mElbow->angleTarget = elbowAngle;

  mBase->stepMoving = abs(mBase->anglePrevious - mBase->angleTarget) / stepNumber;
  mShoulder->stepMoving = abs(mShoulder->anglePrevious - mShoulder->angleTarget) / stepNumber;
  mElbow->stepMoving = abs(mElbow->anglePrevious - mElbow->angleTarget) / stepNumber;

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
    String outString = "ServosAngle : base = " + String(mBase->angleNext) + " shoulder = " + String(mShoulder->angleNext) +  "elbow =" +  String(mElbow->angleNext); 
    Serial.println(outString);
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
