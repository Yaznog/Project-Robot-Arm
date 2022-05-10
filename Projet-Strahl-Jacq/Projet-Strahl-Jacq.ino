#include <Servo.h>
#include <Math.h>
#include "Robot.h"
#include "Nunchuk.h"
#include "Braccio.h"

//#define MATLAB
#define DEBUG

#define X_FROM_LOW -90
#define X_FROM_HIGH 104
#define X_TO_LOW 50
#define X_TO_HIGH 220

#define Y_FROM_LOW -96
#define Y_FROM_HIGH 94
#define Y_TO_LOW -85
#define Y_TO_HIGH 85

#define BASE_ID 1
#define SHOULDER_ID 4
#define ELBOW_ID 3
#define WRISTVER_ID 2
#define WRISTROT_ID 6
#define GRIPPER_ID 5

#define MAX_ANGLE_GRIPPER 110
#define MIN_ANGLE_GRIPPER 35

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
float angleGripper = 90;

void setup() 
{  
  Serial.begin(115200); 
  Wire.begin();
  nunchuk_init();
  Braccio.begin();
  robot = new Robot();
  delay(2000);
}

void loop() 
{
  //Serial.println("loop");
  if(nunchuk_read())
  {
    GetNunchukData();
  }
  robot->MoveWristToCoordinate(nunchuk.x, nunchuk.y, 15.0);
  if(nunchuk.buttonZ && angleGripper < MAX_ANGLE_GRIPPER) angleGripper += 5.0;
  else if (nunchuk.buttonC && angleGripper > MIN_ANGLE_GRIPPER) angleGripper -= 5.0;
  robot->MoveHandToAngle(MaxAngle(nunchuk.angleY)+90.0, -(MaxAngle(nunchuk.angleX)-90.0), angleGripper);
  
  /*robot->MoveOneServoToAngleArm(BASE_ID, (uint8_t)100);*/
  /*robot->MoveOneServoToAngleArm(ELBOW_ID, (uint8_t)70);
  delay(10000);
  robot->MoveOneServoToAngleArm(ELBOW_ID, (uint8_t)90);
  delay(10000);
  robot->MoveOneServoToAngleArm(ELBOW_ID, (uint8_t)110);
  delay(10000);*/
  //DoCircle(10, 10, 1000, 10, 5);
  //DoCircle(50.0, 10.0, 50, 1);
  //delay(10000);
  delay(50);
}

void DoCircle(float radius, float z, uint8_t stepNumber, uint8_t rotationNumber) {
  float stepAngle = 2*PI/stepNumber;

  for(uint8_t rotation=0;rotation<rotationNumber;rotation++)
  {
    for(uint8_t i=0;i<stepNumber;i++)
    {
      //float x = radius*cos( ((float)i/stepNumber)*2*PI ) + 2*radius;
      float x = radius*cos( ((float)i/stepNumber)*2*PI ) + (X_TO_HIGH + X_TO_LOW)/2.0;
      float y = radius*sin( ((float)i/stepNumber)*2*PI );
#ifdef MATLAB
      Serial.print(x);
      Serial.print(" ");
      Serial.print(y);
      Serial.println(";");
#endif
      robot->MoveWristToCoordinate(x, y, z);
            delay(100);
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

float MaxAngle(float angle) {
  if (angle < (-180.0)) return (-180.0);
  if (angle > (180.0)) return (180.0);
  return angle;
}
