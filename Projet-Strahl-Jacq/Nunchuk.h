#ifndef NUNCHUK_H
#define NUNCHUK_H

#include <Wire.h>

// Calibration accelerometer values, depends on your Nunchuk
#define NUNCHUK_ACCEL_X_ZERO 512
#define NUNCHUK_ACCEL_Y_ZERO 512
#define NUNCHUK_ACCEL_Z_ZERO 512

// Calibration joystick values
#define NUNCHUK_JOYSTICK_X_ZERO 127
#define NUNCHUK_JOYSTICK_Y_ZERO 128

// Whether to disable encryption. Enabling encryption means that every packet must be decrypted, which wastes cpu cycles. Cheap Nunchuk clones have problems with the encrypted init sequence, so be sure you know what you're doing
#define NUNCHUK_DISABLE_ENCRYPTION

// The Nunchuk I2C address
#define NUNCHUK_ADDRESS 0x52

#if ARDUINO >= 100
  #define I2C_READ() Wire.read()
  #define I2C_WRITE(x) Wire.write(x)
#else
  #define I2C_READ() Wire.receive()
  #define I2C_WRITE(x) Wire.send(x)
#endif

#define I2C_START(x) Wire.beginTransmission(x)
#define I2C_STOP() Wire.endTransmission(true)

uint8_t nunchuk_data[6];
uint8_t nunchuk_cali[16];

#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
static void nunchuk_init_power() 
{
  // Add power supply for port C2 (GND) and C3 (PWR)
  PORTC &= ~_BV(PORTC2);
  PORTC |= _BV(PORTC3);
  DDRC |= _BV(PORTC2) | _BV(PORTC3);
  //delay(100);
}
#endif

static void nunchuk_init() 
{
  
#ifdef NUNCHUK_DISABLE_ENCRYPTION
    I2C_START(NUNCHUK_ADDRESS);
    I2C_WRITE(0xF0);
    I2C_WRITE(0x55);
    I2C_STOP();
    I2C_START(NUNCHUK_ADDRESS);
    I2C_WRITE(0xFB);
    I2C_WRITE(0x00);
    I2C_STOP();
#else
    I2C_START(NUNCHUK_ADDRESS);
    I2C_WRITE(0x40);
    I2C_WRITE(0x00);
    I2C_STOP();
#endif

#ifdef NUNCHUK_DEBUG
    Serial.print("Ident: "); // 0xA4200000 for Nunchuck, 0xA4200101 for Classic, 0xA4200402 for Balance

    I2C_START(NUNCHUK_ADDRESS);
    I2C_WRITE(0xFA);
    I2C_STOP();

    Wire.requestFrom(NUNCHUK_ADDRESS, 6);
    for (uint8_t i = 0; i < 6; i++) {
        if (Wire.available()) {
            Serial.print(I2C_READ(), HEX);
            Serial.print(" ");
        }
    }
    I2C_STOP();
    Serial.println("");
    //delay(100);
#endif

}

/**
 * Decodes a byte if encryption is used
 * @param x The byte to be decoded
 */
static inline uint8_t nunchuk_decode_byte(uint8_t x) {
#ifdef NUNCHUK_DISABLE_ENCRYPTION
    return x;
#else
    return (x ^ 0x17) + 0x17;
#endif
}

/**
 * Central function to read a full chunk of data from Nunchuk
 * @return A boolean if the data transfer was successful
 */
static uint8_t nunchuk_read() 
{
  uint8_t i;
  Wire.requestFrom(NUNCHUK_ADDRESS, 6);
    for (i = 0; i < 6 && Wire.available(); i++) {
        nunchuk_data[i] = nunchuk_decode_byte(I2C_READ());
    }
    I2C_START(NUNCHUK_ADDRESS);
    I2C_WRITE(0x00);

    I2C_STOP();
    return i == 6;
}

// Buttons------------------------------------------------------

static uint8_t nunchuk_buttonZ() 
{
  return (~nunchuk_data[5] >> 0) & 1;
}

static uint8_t nunchuk_buttonC() 
{
  return (~nunchuk_data[5] >> 1) & 1;
}

// Joystick------------------------------------------------------

static uint8_t nunchuk_joystickX_raw() 
{
  return nunchuk_data[0];
}

static uint8_t nunchuk_joystickY_raw() 
{
  return nunchuk_data[1];
}

// Calibrated Joystick--------------------------------------------

static int16_t nunchuk_joystickX() 
{
  return (int16_t) nunchuk_joystickX_raw() - (int16_t) NUNCHUK_JOYSTICK_X_ZERO;
}

static int16_t nunchuk_joystickY() 
{
  return (int16_t) nunchuk_joystickY_raw() - (int16_t) NUNCHUK_JOYSTICK_Y_ZERO;
}

// Accelerometer--------------------------------------------------

static uint16_t nunchuk_accelX_raw() 
{
  return ((uint16_t) nunchuk_data[2] << 2) | ((nunchuk_data[5] >> 2) & 3);
}

static uint16_t nunchuk_accelY_raw() 
{
  return ((uint16_t) nunchuk_data[3] << 2) | ((nunchuk_data[5] >> 4) & 3);
}

static uint16_t nunchuk_accelZ_raw() 
{
  return ((uint16_t) nunchuk_data[4] << 2) | ((nunchuk_data[5] >> 6) & 3);
}

// Calibrated Accelerometer------------------------------------------

static int16_t nunchuk_accelX() 
{
  return (int16_t) nunchuk_accelX_raw() - (int16_t) NUNCHUK_ACCEL_X_ZERO;
}

static int16_t nunchuk_accelY() 
{
  return (int16_t) nunchuk_accelY_raw() - (int16_t) NUNCHUK_ACCEL_Y_ZERO;
}

static int16_t nunchuk_accelZ() 
{
  return (int16_t) nunchuk_accelZ_raw() - (int16_t) NUNCHUK_ACCEL_Z_ZERO;
}


// Calculates the pitch angle THETA around y-axis of the Nunchuk in radians
static float nunchuk_pitch() 
{ // tilt-y
  return atan2((float) nunchuk_accelY(), (float) nunchuk_accelZ());
}

// Calculates the roll angle PHI around x-axis of the Nunchuk in radians
static float nunchuk_roll() 
{ // tilt-x
  return atan2((float) nunchuk_accelX(), (float) nunchuk_accelZ());
}

static int16_t joystick_DeadZone(int16_t value)
{
  int16_t deadzone_Min = 10;
  int16_t deadzone_Max = 90;
  
  if( value<deadzone_Min && value>-deadzone_Min ) return 0;
  if(value>deadzone_Max) return(deadzone_Max);
  if(value<-deadzone_Max) return(-deadzone_Max);
  return(value);
}

static boolean JoystickForward()
{
  
  //Serial.println("joystick fo");
  int16_t nunchuk_X_Value = joystick_DeadZone(nunchuk_joystickX());
  int16_t nunchuk_Y_Value = joystick_DeadZone(nunchuk_joystickY());
  
  float angle = atan2(nunchuk_Y_Value, nunchuk_X_Value);

  if( (angle <= (3*PI/4) ) && (angle >= (PI/4)) ){
      //Serial.println("true");
    return true;
  }
      //Serial.println("false");
  return false;
}

static boolean JoystickRearward()
{
  int16_t nunchuk_X_Value = joystick_DeadZone(nunchuk_joystickX());
  int16_t nunchuk_Y_Value = joystick_DeadZone(nunchuk_joystickY());
  
  float angle = atan2(nunchuk_Y_Value, nunchuk_X_Value);
  
  if( (angle <= (-PI/4) ) && (angle >= (-3*PI/4)) ) return true;
  return false;
}

static boolean JoystickLeftSide()
{
  int16_t nunchuk_X_Value = joystick_DeadZone(nunchuk_joystickX());
  int16_t nunchuk_Y_Value = joystick_DeadZone(nunchuk_joystickY());
  
  float angle = atan2(nunchuk_Y_Value, nunchuk_X_Value);

  if( (angle <= (-3*PI/4) ) || (angle >= (3*PI/4)) ) return true;
  return false;
}

static boolean JoystickRightSide()
{

  int16_t nunchuk_X_Value = joystick_DeadZone(nunchuk_joystickX());
  int16_t nunchuk_Y_Value = joystick_DeadZone(nunchuk_joystickY());
  
  float angle = atan2(nunchuk_Y_Value, nunchuk_X_Value);

  if( (angle <= (PI/4) ) && (angle >= (-PI/4)) && (nunchuk_X_Value!=0 || nunchuk_Y_Value!=0)) return true;
  return false;
}


#endif
