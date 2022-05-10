#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <Servo.h>
/*
#define BASE_ANGLE_INIT 90
#define BASE_ANGLE_MAX 180
#define BASE_ANGLE_MIN 0
#define BASE_RANGE 70
#define BASE_ID 1
#define BASE_PIN 11
    
#define SHOULDER_ANGLE_INIT 90
#define SHOULDER_ANGLE_MAX 180
#define SHOULDER_ANGLE_MIN 0
#define SHOULDER_RANGE 115
#define SHOULDER_ID 4
#define SHOULDER_PIN 6

#define ELBOW_ANGLE_INIT 90
#define ELBOW_ANGLE_MAX 180
#define ELBOW_ANGLE_MIN 0
#define ELBOW_RANGE 125
#define ELBOW_ID 3
#define ELBOW_PIN 9*/

#ifndef SERVOMOTOR
#define SERVOMOTOR

struct Servomotor{
  uint8_t angle;
  uint8_t angleTarget;
  uint8_t anglePrevious;
  uint8_t angleNext;
  uint8_t angleInit;
  uint8_t angleMax;
  uint8_t angleMin;
  uint8_t stepMoving;
  uint8_t range;
  int8_t pin = -1;
};
#endif

#ifndef COORDINATE
#define COORDINATE
struct Coordinate{
  float x;
  float y;
  float z;
  float module;
  float argument;
};
#endif

class Arm
{
  public:

    Arm();
    ~Arm();

    void ArmInit();
    void AttachServos();
    void InitPositionServos();
    void SetPinServo();
    void SetRangeServos();
    void CalibrateServos();
    
    void MoveToTarget();
    void MoveToTarget(uint16_t timeDelay);
    void MoveOneServoToAngle(uint8_t servoID, uint8_t angle);
    void MoveOneServoToAngle(uint8_t servoID, uint8_t angle, uint16_t timeDelay);
    void MoveServosToAngle(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle);
    void MoveServosToAngle(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle, uint16_t timeDelay); //timeDelay in millisecond

    void SetCoordinateTarget(float x, float y, float z);
    void SetCoordinatePolarTarget(float module, float argument, float z);
    
    //float ServoMaxAngle(Servomotor *servo, uint8_t angle);
    float GetServoBaseAngle();
    float GetServoShoulderAngle();
    float GetServoElbowAngle();
    //float MaxAngle(float value);

    uint8_t GetBaseAngle();
    uint8_t GetShoulderAngle();
    uint8_t GetElbowAngle();
    
  private:    

const int8_t BASE_ANGLE_INIT = 90;
const int8_t BASE_ANGLE_MAX = 180;
const int8_t BASE_ANGLE_MIN = 0;
const int8_t BASE_RANGE = 71;
const int8_t BASE_ID = 1;
const int8_t BASE_PIN = 11;
    
const int8_t SHOULDER_ANGLE_INIT = 90;
const int8_t SHOULDER_ANGLE_MAX = 180;
const int8_t SHOULDER_ANGLE_MIN = 0;
const int8_t SHOULDER_RANGE = 125;
const int8_t SHOULDER_ID = 4;
const int8_t SHOULDER_PIN = 6;

const int8_t ELBOW_ANGLE_INIT = 90;
const int8_t ELBOW_ANGLE_MAX = 180;
const int8_t ELBOW_ANGLE_MIN = 0;
const int8_t ELBOW_RANGE = 125;
const int8_t ELBOW_ID = 3;
const int8_t ELBOW_PIN = 9;

    Servomotor mBase;
    Servomotor mShoulder;
    Servomotor mElbow;

    Coordinate mTarget;
    
    Servo mServoBase;
    Servo mServoShoulder;
    Servo mServoElbow;
};

#endif
