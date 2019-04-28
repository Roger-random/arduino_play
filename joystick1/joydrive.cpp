/* 
  *  JoyDrive = Joystick Drive
  * ------------
  *
  * Reads two analog pins that are connected to potentiometers representing two
  * axis of a joystick. One axis represents steering angle, one axis represents
  * velocity. Each is normalized within a range from -100 to 100, so the sign
  * represents direction and number represents magnitude as a percentage.
  * 
  */

#include "joydrive.h"

JoyDrive::JoyDrive(int steeringPin, int velocityPin, int buttonPin)
{
  _steeringPin = steeringPin;
  _velocityPin = velocityPin;
  _buttonPin = buttonPin;
  pinMode(_buttonPin, INPUT);
  digitalWrite(_buttonPin, HIGH);

  // Set default values for other options
  _deadZone = 25;
  _steeringInvert = false;
  _velocityInvert = false;
}

void JoyDrive::invertSteering(bool invert)
{
  _steeringInvert = invert;
}

void JoyDrive::invertVelocity(bool invert)
{
  _velocityInvert = invert;
}

void JoyDrive::setDeadZone(int deadZone)
{
  _deadZone = deadZone;
}

bool JoyDrive::getButton()
{
  return (digitalRead(_buttonPin) == HIGH);
}

int JoyDrive::getSteering()
{
  // analogRead returns between 0 and 1023
  int vRaw = analogRead(_steeringPin);

  return normalized(vRaw, _steeringInvert);
}

int JoyDrive::getVelocity()
{
  // analogRead returns between 0 and 1023
  int vRaw = analogRead(_velocityPin);

  return normalized(vRaw, _velocityInvert);
}

int JoyDrive::normalized(int raw, bool invert)
{
  int range = ANALOG_MID-_deadZone;
  float sign = 0.0;
  int subset = 0;
  
  if (raw > ANALOG_MID+_deadZone)
  {
    sign = 100.0;
    subset = raw - (ANALOG_MID+_deadZone);
  }
  else if (raw < range)
  {
    sign = -100.0;
    subset = range - raw;
  }
  else
  {
    sign = 100.0;
    subset = 0;
  }

  if (invert)
  {
    sign = sign * -1;
  }

  return sign * subset / range;
}
