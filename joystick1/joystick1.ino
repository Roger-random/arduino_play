#include "Arduino.h"
#include "joydrive.h"
#include "lewansoul.h"

#define STEERING_PIN 1
#define VELOCITY_PIN 0
#define BUTTON_PIN 2

JoyDrive jd(STEERING_PIN, VELOCITY_PIN,BUTTON_PIN);
LewanSoul lss(1);

typedef struct RoverWheel
{
  float x;
  float y;
  int rollServoId;
  bool rollServoInverted;
  int steerServoId;
  float steerTrim;
} RoverWheel;

const RoverWheel Chassis[] = {
  // front left
  {
    -9.125, // x
    11.375, // y
    25,     // roll ID
    false,  // roll inverted
    23,     // steer ID
    -4      // steer trim
  },
  // front right
  {
     9.125, // x
    11.375, // y
    27,     // roll ID
    true,   // roll inverted
    29,     // steer ID
    -4      // steer trim
  },
  // mid left
  {
    -10.375,// x
    0,      // y
    21,     // roll ID
    false,  // roll inverted
    -1,     // steer ID
    0       // steer trim
  },
  // mid right
  {
    10.375, // x
    0,      // y
    22,     // roll ID
    true,   // roll inverted
    -1,     // steer ID
    0       // steer trim
  },
  // rear left
  {
    -9,     // x
    -10,    // y
    20,     // roll ID
    false,  // roll inverted
    24,     // steer ID
    0       // steer trim
  },
  // rear right
  {
     9,     // x
    -10,    // y
    28,     // roll ID
    true,   // roll inverted
    26,     // steer ID
    2       // steer trim
  }
};

void setup()
{
  lss.setup();
}

void loop()
{
  int steering;
  int velocity;
  int wheel;
  int invert;

  delay(100);
  steering = jd.getSteering();
  delay(100);
  velocity = jd.getVelocity();

  for (wheel = 0; wheel < 6; wheel++)
  {
    if (Chassis[wheel].rollServoInverted)
    {
      invert = -1;
    }
    else
    {
      invert = 1;
    }
    lss.spinAt(Chassis[wheel].rollServoId, velocity * invert);
    if (Chassis[wheel].steerServoId > 1)
    {
      lss.moveTo(Chassis[wheel].steerServoId, Chassis[wheel].steerTrim);
    }
  }
}

/* JoyDrive test program
 
void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.print("Joystick velocity=");
  Serial.print(jd.getVelocity());
  Serial.print(" steering=");
  Serial.print(jd.getSteering());
  Serial.print(" button=");
  Serial.println(jd.getButton());
}

*/
