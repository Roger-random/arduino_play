#include <math.h>
#include "Arduino.h"
#include "joydrive.h"
#include "lewansoul.h"

//#define LEWANSOUL 1
#define PRINTCMD 1

#define STEERING_PIN 1
#define VELOCITY_PIN 0
#define BUTTON_PIN 2

JoyDrive jd(STEERING_PIN, VELOCITY_PIN,BUTTON_PIN);

#ifdef LEWANSOUL
LewanSoul lss(1);
#endif

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
    2       // steer trim
  },
  // rear right
  {
     9,     // x
    -10,    // y
    28,     // roll ID
    true,   // roll inverted
    26,     // steer ID
    4       // steer trim
  }
};

// Index into Chassis[] used for calculating steering angles
#define FRONT_LEFT 0
#define FRONT_RIGHT 1
#define MID_LEFT 2

// Using information above, the maximum steering angle for a front wheel
float maxSteering;

// Store calculation of servo angle and speed before we send them as commands
typedef struct ServoCommand {
  float angle;
  float speed;
} ServoCommand;

ServoCommand servoCommands[6];

void setup()
{
  float opposite;
  float adjacent;

#ifdef LEWANSOUL
  // LewanSoul serial servo code is taking over Serial port.
  lss.setup();
#endif

#ifdef PRINTCMD
  // Use serial port to print our calculated commands
  Serial.begin(9600);
#endif

  // Calculate maximum steering angle
  adjacent = Chassis[MID_LEFT].x - Chassis[FRONT_LEFT].x;
  opposite = Chassis[FRONT_LEFT].y;
  maxSteering = abs(atan(opposite/adjacent)*180/M_PI);
}

void loop()
{
  int steering;
  int velocity;
  int wheel;
  float calcSteer;
  float turnRadius;

  int invert;

  delay(100);
  steering = jd.getSteering();
  delay(100);
  velocity = jd.getVelocity();

  if (steering > 0)
  {
    // Front right wheel is our reference
    servoCommands[FRONT_RIGHT].angle = maxSteering * steering / 100.0;
  }
  else if (steering < 0)
  {
    // Front left wheel is our reference
    servoCommands[FRONT_LEFT].angle = maxSteering * steering / 100.0;
  }
  else
  {
    turnRadius = 0;
  }

  for (wheel = 0; wheel < 6; wheel++)
  {
    if (Chassis[wheel].steerServoId != -1)
    {
      if (steering > 0 && wheel != FRONT_RIGHT)
      {
        servoCommands[wheel].angle = 0;
      }
      else if (steering < 0 && wheel != FRONT_LEFT)
      {
        servoCommands[wheel].angle = 0;
      }
      else if (steering == 0)
      {
        servoCommands[wheel].angle = 0;
      }
    }
    servoCommands[wheel].speed = velocity;
  }

#ifdef LEWANSOUL
  // All angles and speeds calculated, send the commands accounting
  // for steering trim offset and inverting speed where needed
  for (wheel = 0; wheel < 6; wheel++)
  {
    if (Chassis[wheel].steerServoId != -1)
    {
      lss.moveTo(Chassis[wheel].steerServoId, servoCommands[wheel].angle + Chassis[wheel].steerTrim);
    }

    if (Chassis[wheel].rollServoInverted)
    {
      invert = -1;
    }
    else
    {
      invert = 1;
    }
    lss.spinAt(Chassis[wheel].rollServoId, servoCommands[wheel].speed * invert);
  }
#endif

#ifdef PRINTCMD
  for (wheel = 0; wheel < 6; wheel++)
  {
    Serial.print(" W");
    Serial.print(wheel);
    Serial.print(" ");
    if (Chassis[wheel].steerServoId != -1)
    {
      Serial.print(servoCommands[wheel].angle + Chassis[wheel].steerTrim);
      Serial.print("deg ");
    }

    if (Chassis[wheel].rollServoInverted)
    {
      invert = -1;
    }
    else
    {
      invert = 1;
    }
    Serial.print(servoCommands[wheel].speed * invert);
    Serial.print("pct ");
  }
  Serial.println();
#endif

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
