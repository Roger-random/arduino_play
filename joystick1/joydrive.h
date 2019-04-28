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

#ifndef joydrive_h
#define joydrive_h

#include "arduino.h"

#define ANALOG_MAX 1023
#define ANALOG_MID 512

class JoyDrive
{
  public:
    JoyDrive(int steeringPin, int velocityPin, int buttonPin);

    int getSteering();
    int getVelocity();
    bool getButton();

    void invertSteering(bool invert);
    void invertVelocity(bool invert);

    void setDeadZone(int deadZone);
  private:
    int normalized(int raw, bool invert);

    int _steeringPin;
    int _velocityPin;
    int _buttonPin;

    int _deadZone;

    bool _steeringInvert;
    bool _velocityInvert;
};
#endif // joydrive_h
