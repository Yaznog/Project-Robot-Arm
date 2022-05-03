#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <Servo.h>

#define BASE_ANGLE_INIT 90
#define BASE_ANGLE_MAX 180
#define BASE_ANGLE_MIN 0
#define BASE_RANGE 100
#define BASE_ID 1
    
#define SHOULDER_ANGLE_INIT 90
#define SHOULDER_ANGLE_MAX 180
#define SHOULDER_ANGLE_MIN 0
#define SHOULDER_RANGE 200
#define SHOULDER_ID 2    

#define ELBOW_ANGLE_INIT 90
#define ELBOW_ANGLE_MAX 180
#define ELBOW_ANGLE_MIN 0
#define ELBOW_RANGE 200
#define ELBOW_ID 3

#ifndef SERVOMOTOR
#define SERVOMOTOR

struct Servomotor{
  Servo *servo;
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

    Arm(int8_t pinBase, int8_t pinShoulder, int8_t pinElbow);
    ~Arm();

    void ArmInit();
    void AttachServos();
    void InitPositionServos();
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
    
    float ServoMaxAngle(Servomotor *servo, uint8_t angle);
    float GetServoBaseAngle();
    float GetServoShoulderAngle();
    float GetServoElbowAngle();
    float MaxAngle(float value);
    float RadToDegree(float angle);

    uint8_t GetBaseAngle();
    uint8_t GetShoulderAngle();
    uint8_t GetElbowAngle();
    
  private:    
    
    Servomotor *mBase;
    Servomotor *mShoulder;
    Servomotor *mElbow;

    Coordinate *mTarget;
};

#endif
