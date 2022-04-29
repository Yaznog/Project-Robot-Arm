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
    void UpdateServoLocation();

  private:

    Arm *arm;
    //Hand *hand;
};

#endif
