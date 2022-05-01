#ifndef HAND_H
#define HAND_H

#include <Arduino.h>
#include <Servo.h>

#define WRISTVER_ANGLE_INIT 90
#define WRISTVER_ANGLE_MAX 180
#define WRISTVER_ANGLE_MIN 0
#define WRISTVER_RANGE 100
    
#define WRISTROT_ANGLE_INIT 90
#define WRISTROT_ANGLE_MAX 180
#define WRISTROT_ANGLE_MIN 0
#define WRISTROT_RANGE 200
    
#define GRIPPER_ANGLE_INIT 90
#define GRIPPER_ANGLE_MAX 180
#define GRIPPER_ANGLE_MIN 0
#define GRIPPER_RANGE 200

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

class Hand
{
  public:

    Hand(int8_t pinWristVer, int8_t pinWristRot, int8_t pinGripper);
    ~Hand();

    void HandInit();
    void AttachServos();
    void InitPositionServos();
    void SetRangeServos();
    void CalibrateServos();
    
    //void MoveToTarget();
    //void MoveToTarget(uint16_t timeDelay);
    //void MoveServosToAngle(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle);
    //void MoveServosToAngle(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle, uint16_t timeDelay); //timeDelay in millisecond

    //void SetCoordinateTarget(float x, float y, float z);
    //void SetCoordinatePolarTarget(float module, float argument, float z);
    
    //float ServoMaxAngle(Servomotor *servo, uint8_t angle);
    //float GetServoBaseAngle();
    //float GetServoShoulderAngle();
    //float GetServoElbowAngle();
    float MaxAngle(float value);
    float RadToDegree(float angle);

    uint8_t GetWristVerAngle();
    uint8_t GetWristRotAngle();
    uint8_t GetGripperAngle();
    
  private:    
    
    Servomotor *mWristVer;
    Servomotor *mWristRot;
    Servomotor *mGripper;
};

#endif
