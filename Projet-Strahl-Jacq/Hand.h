#ifndef HAND_H
#define HAND_H

#include <Arduino.h>
#include <Servo.h>

#define WRISTVER_ANGLE_INIT 90
#define WRISTVER_ANGLE_MAX 180
#define WRISTVER_ANGLE_MIN 0
#define WRISTVER_RANGE 65
#define WRISTVER_ID 2
#define WRISTVER_PIN 10
    
#define WRISTROT_ANGLE_INIT 90
#define WRISTROT_ANGLE_MAX 180
#define WRISTROT_ANGLE_MIN 0
#define WRISTROT_RANGE 33
#define WRISTROT_ID 6
#define WRISTROT_PIN 3
    
#define GRIPPER_ANGLE_INIT 90
#define GRIPPER_ANGLE_MAX 180
#define GRIPPER_ANGLE_MIN 0
#define GRIPPER_RANGE 95
#define GRIPPER_ID 5
#define GRIPPER_PIN 5

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

class Hand
{
  public:

    Hand();
    ~Hand();

    void HandInit();
    void AttachServos();
    void InitPositionServos();
    void SetPinServo();
    void SetRangeServos();
    void CalibrateServos();
    
    //void MoveToTarget();
    //void MoveToTarget(uint16_t timeDelay);
    void MoveServosToAngle(uint8_t wristVerAngle, uint8_t wristRotAngle, uint8_t gripperAngle);
    //void MoveServosToAngle(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle, uint16_t timeDelay); //timeDelay in millisecond

    //void SetCoordinateTarget(float x, float y, float z);
    //void SetCoordinatePolarTarget(float module, float argument, float z);
    
    //float ServoMaxAngle(Servomotor *servo, uint8_t angle);
    //float GetServoBaseAngle();
    //float GetServoShoulderAngle();
    //float GetServoElbowAngle();
    float MaxAngle(float value);

    uint8_t GetWristVerAngle();
    uint8_t GetWristRotAngle();
    uint8_t GetGripperAngle();
    
  private:    

    Servomotor mWristVer;
    Servomotor mWristRot;
    Servomotor mGripper;
    
    Servo mServoWristRot;
    Servo mServoWristVer;
    Servo mServoGripper;
};

#endif
