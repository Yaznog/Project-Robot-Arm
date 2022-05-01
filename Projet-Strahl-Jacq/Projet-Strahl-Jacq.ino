#include <Servo.h>
#include <Math.h>
#include "Robot.h"

//#define MATLAB

Robot* robot;

void setup() 
{  
  Serial.begin(9600); 
  robot = new Robot();
}

void loop() 
{
  DoCircle(10, 10, 500, 10, 1);
  delay(100000);
}

void DoCircle(float radius, float z, uint16_t timeDelay, uint8_t stepNumber, uint8_t rotationNumber)
{
  float stepAngle = 2*PI/stepNumber;

  for(uint8_t rotation=0;rotation<rotationNumber;rotation++)
  {
    for(uint8_t i=0;i<stepNumber;i++)
    {
      float x = radius*cos( ((float)i/stepNumber)*2*PI ) + 2*radius;
      float y = radius*sin( ((float)i/stepNumber)*2*PI );
#ifdef MATLAB
      Serial.print(x);
      Serial.print(" ");
      Serial.print(y);
      Serial.println(";");
#endif
      robot->MoveWristToCoordinate(x, y, z, timeDelay);
    }
  }
}
