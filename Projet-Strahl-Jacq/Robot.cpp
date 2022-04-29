#include "Robot.h"

#define DEBUG

Robot::Robot()
{
  
#ifdef DEBUG
  Serial.println("New Robot");
#endif

  // Create arm (pinBase, pinShoulder, pinElbow)
  arm = new Arm(11, 10, 9);
  // Create hand (pinWrist_ver, pinWrist_rot, pinGripper)
  //hand  = new Hand(5, 6, 3);
}

Robot::~Robot()
{
#ifdef DEBUG
  Serial.println("Robot deleted");
#endif

  //hand->~Hand();
  arm->~Arm();
}

void Robot::UpdateServoLocation()
{
  
}
