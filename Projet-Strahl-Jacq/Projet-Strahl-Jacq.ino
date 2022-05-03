#include <Servo.h>
#include <Math.h>
#include "Robot.h"
#include "Nunchuk.h"

//#define MATLAB
//#define DEBUG

#define X_FROM_LOW -90
#define X_FROM_HIGH 104
#define X_TO_LOW 100
#define X_TO_HIGH 300

#define Y_FROM_LOW -96
#define Y_FROM_HIGH 94
#define Y_TO_LOW -100
#define Y_TO_HIGH 100

#define BASE_ID 1
#define SHOULDER_ID 2
#define ELBOW_ID 3
#define WRISTVER_ID 4
#define WRISTROT_ID 5
#define GRIPPER_ID 6

struct Nunchuk{
  float x;
  float y;
  bool buttonZ;
  bool buttonC;
  float angleX;
  float angleY;
};

Nunchuk nunchuk;
Robot* robot;

void setup() 
{  
  Serial.begin(115200); 
  Wire.begin();
  nunchuk_init();
  robot = new Robot();
}

void loop() 
{
  robot->MoveOneServoToAngleArm(BASE_ID, 90);
  /*
  if(nunchuk_read())
  {
    GetNunchukData();
  }*/
  //robot->MoveWristToCoordinate(nunchuk.x, nunchuk.y, 100.0);
  delay(100);
  //DoCircle(10, 10, 500, 10, 1);
  //DoCircle(10, 10, 10, 1);
  //delay(100000);
}

void DoCircle(float radius, float z, uint8_t stepNumber, uint8_t rotationNumber) {
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
      robot->MoveWristToCoordinate(x, y, z);
    }
  }
}

void DoCircle(float radius, float z, uint16_t timeDelay, uint8_t stepNumber, uint8_t rotationNumber) {
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

void GetNunchukData() {
  nunchuk.x = map(nunchuk_joystickX(), X_FROM_LOW, X_FROM_HIGH, X_TO_LOW, X_TO_HIGH);
  nunchuk.y = map(nunchuk_joystickY(), Y_FROM_LOW, Y_FROM_HIGH, Y_TO_LOW, Y_TO_HIGH);
  nunchuk.buttonZ = nunchuk_buttonZ();
  nunchuk.buttonC = nunchuk_buttonC();
  nunchuk.angleX = nunchuk_roll()*180.0/PI;
  nunchuk.angleY = nunchuk_pitch()*180.0/PI;

#ifdef DEBUG
  Serial.print("X = ");
  Serial.print(nunchuk.x);
  Serial.print(" Y = ");
  Serial.print(nunchuk.y);
  
  Serial.print(" Bouton Z = ");
  Serial.print(nunchuk.buttonZ);
  Serial.print(" Bouton C = ");
  Serial.print(nunchuk.buttonC);
  
  Serial.print(" Angle X = ");
  Serial.print(nunchuk.angleX);
  Serial.print(" Angle Y = ");
  Serial.println(nunchuk.angleY);
#endif
}
