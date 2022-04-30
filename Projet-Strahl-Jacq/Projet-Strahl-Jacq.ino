#include <Servo.h>
#include <Math.h>
#include "Robot.h"

Robot* robot;

void setup() 
{  
  Serial.begin(9600); 
  robot = new Robot();
}

void loop() 
{
  DoCircle(7.5, 5, 500, 20, 1);
  delay(10000);
}

void DoCircle(float radius, float z, uint16_t timeDelay, uint8_t stepNumber, uint8_t rotationNumber)
{
  float stepAngle = 2*PI/stepNumber;

  for(uint8_t rotation=0;rotation<rotationNumber;rotation++)
  {
    for(uint8_t i=0;i<stepNumber;i++)
    {
      float x = radius*cos( ((float)i/stepNumber)*2*PI ) + 1.5*radius;
      float y = radius*sin( ((float)i/stepNumber)*2*PI );
      robot->MoveWristToCoordinate(x, y, z, timeDelay);
    }
  }
}
