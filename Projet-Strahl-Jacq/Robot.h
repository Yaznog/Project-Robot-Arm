#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "Arm.h"
#include "Hand.h"

// You should set begin(SOFT_START_DISABLED) if you are using the Arm Robot shield V1.6
#define SOFT_START_DISABLED    -999

//The default value for the soft start
#define SOFT_START_DEFAULT    0

//The software PWM is connected to PIN 12. You cannot use the pin 12 if you are using
//a Braccio shield V4 or newer
#define SOFT_START_CONTROL_PIN  12

//Low and High Limit Timeout for the Software PWM
#define LOW_LIMIT_TIMEOUT 2000
#define HIGH_LIMIT_TIMEOUT 6000

class Robot
{
  public:

    Robot();
    ~Robot();

    unsigned int RobotInit(int soft_start_level=SOFT_START_DEFAULT);
    void _softStart(int soft_start_level);
    void _softwarePWM(int high_time, int low_time);

    void MoveOneServoToAngleArm(uint8_t servoID, uint8_t angle);
    void MoveOneServoToAngleArm(uint8_t servoID, uint8_t angle, uint16_t timeDelay);
    void MoveWristToCoordinate(float x, float y, float z);
    void MoveWristToCoordinate(float x, float y, float z, uint16_t timeDelay);
    void MoveWristToCoordinatePolar(float module, float argument, float z);
    void MoveWristToCoordinatePolar(float module, float argument, float z, uint16_t timeDelay);

    void InitPositionArm();
    void InitPositionHand();
      
    void CalibrateArm();
    void CalibrateHand();

  private:

    Arm *mArm;
    Hand *mHand;
};

#endif
