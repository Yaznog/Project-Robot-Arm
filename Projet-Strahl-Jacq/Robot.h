#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Servo.h>
#include "Braccio.h"
#include "Arm.h"
#include "Hand.h"

class Robot
{
  public:

    Robot();
    ~Robot();

    void RobotInit();

    void MoveOneServoToAngleArm(uint8_t servoID, uint8_t angle);
    void MoveOneServoToAngleArm(uint8_t servoID, uint8_t angle, uint16_t timeDelay);
    void MoveWristToCoordinate(float x, float y, float z);
    void MoveWristToCoordinate(float x, float y, float z, uint16_t timeDelay);
    void MoveWristToCoordinatePolar(float module, float argument, float z);
    void MoveWristToCoordinatePolar(float module, float argument, float z, uint16_t timeDelay);
    void MoveHandToAngle(uint8_t wristVerAngle, uint8_t wristRotAngle, uint8_t gripperAngle);

    void InitPositionArm();
    void InitPositionHand();
      
    void CalibrateArm();
    void CalibrateHand();
    
  private:
    
    Arm *mArm;
    Hand *mHand;
};

#endif
