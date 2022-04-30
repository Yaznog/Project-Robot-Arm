#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Arm.h"
//#include "Hand.h"

class Robot
{
  public:

    Robot();
    ~Robot();
    
    void MoveWristToCoordinate(float x, float y, float z);
    void MoveWristToCoordinate(float x, float y, float z, uint16_t timeDelay);
    void MoveWristToCoordinatePolar(float module, float argument, float z);
    void MoveWristToCoordinatePolar(float module, float argument, float z, uint16_t timeDelay);

    void CalibrateArm();
    void CalibrateHand();

  private:

    Arm *mArm;
    //Hand *mHand;
};

#endif
