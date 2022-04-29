#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <Servo.h>

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

#define BASE_ANGLE_INIT 90
#define SHOULDER_ANGLE_INIT 90
#define ELBOW_ANGLE_INIT 90

struct Servomotor
{
  Servo *servo;
  uint8_t angle;
  uint8_t angleTarget;
  uint8_t anglePrevious;
  uint8_t angleNext;
  uint8_t angleInit;
  uint8_t stepMoving;
  //bool FLAG_MOVING = false;
  int8_t pin = -1;
};

struct Coordinate
{
  float x;
  float y;
  float z;
  float module;
  float argument;
};

class Arm
{
  public:

    Arm(int8_t pinBase, int8_t pinShoulder, int8_t pinElbow);
    ~Arm();

    unsigned int ArmInit(int soft_start_level=SOFT_START_DEFAULT);
    void _softStart(int soft_start_level);
    void _softwarePWM(int high_time, int low_time);
    void AttachServos();
    void InitPositionServos();
    void CalibrateServos();

    void MoveServosToAngleTime(uint8_t baseAngle, uint8_t shoulderAngle, uint8_t elbowAngle, uint16_t timeDelay); //timeDelay in millisecond

  private:
    /*
        int8_t mPinBase;
        int8_t mPinShoulder;
        int8_t mPinElbow;

        Servo *mBase;
        Servo *mShoulder;
        Servo *mElbow;*/

    Servomotor *mBase;
    Servomotor *mShoulder;
    Servomotor *mElbow;
    /*
        const unsigned int mBaseLength = 100;
        const unsigned int mShoulderLength = 100;
        const unsigned int mElbowLength = 100;

        const int8_t mServoTopForbiddenValue = 10;
        const int8_t mServoMidForbiddenValue = 30;
        const int8_t mServoBotForbiddenValue = 30;

        float mXCoordinate;
        float mYCoordinate;
        float mZCoordinate;
        float mModuleCoordinate;
        float mArgumentCoordinate;

        int mEtatMarche = 0;*/
};

#endif
