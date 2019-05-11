#include <math.h>
#include "Arduino.h"
#include "joydrive.h"
#include "lewansoul.h"

// Output select - only one of the following should be uncommented
#define LEWANSOUL 1
//#define PRINTCMD 1
// Output select end

#define STEERING_PIN 1
#define VELOCITY_PIN 0
#define INPLACE_BUTTON 2 // Button to trigger turn-in-place mode.

JoyDrive jd(STEERING_PIN, VELOCITY_PIN);

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
// Also stores turning radius (hypotenuse in trig calculations) which affects speed.
typedef struct ServoCommand {
  float angle;
  float radius;
  float speed;
} ServoCommand;

ServoCommand servoCommands[6];

// Use the servoCommands[].radius values to scale all servoCommands[].speed relative
// to a given velocity.
void speedFromRadius(float maxSpeed)
{
  int wheel; // iterator for wheels on a chassis
  float maxRadius = 0.0; // Turning radius of wheel furthest from turn center

  for (wheel = 0; wheel < 6; wheel++)
  {
    if (abs(servoCommands[wheel].radius) > maxRadius)
    {
      maxRadius = abs(servoCommands[wheel].radius);
    }
  }
  // Max radius found, now scale all wheel speeds so those with max radius spins
  // at commanded velocity and other wheels at slower speed based on radius ratio.
  for (wheel = 0; wheel < 6; wheel++)
  {
    servoCommands[wheel].speed = (servoCommands[wheel].radius/maxRadius)*maxSpeed;
  }
}

void setup()
{
  float opposite;
  float adjacent;

  // Configure button pins
  pinMode(INPLACE_BUTTON, INPUT);
  digitalWrite(INPLACE_BUTTON, HIGH); // Button pulls LOW when pressed

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
  maxSteering = abs(atan(opposite/adjacent)*180.0/M_PI);
}

void loop()
{
  int steering;
  int velocity;
  int wheel;
  float turnCenterX;
  float inRadians;

  int invert;
  int referenceWheel;

  bool turnInPlace = (digitalRead(INPLACE_BUTTON) == LOW);
  delay(100);
  steering = jd.getSteering();
  delay(100);
  velocity = jd.getVelocity();

  if (turnInPlace)
  {
    for (wheel = 0; wheel < 6; wheel++)
    {
      // Calculate angles to point at center
      if (Chassis[wheel].y == 0)
      {
        servoCommands[wheel].angle = 0;
        servoCommands[wheel].radius = abs(Chassis[wheel].x);
      }
      else
      {
        servoCommands[wheel].angle = -atan(Chassis[wheel].y/Chassis[wheel].x)*180.0/M_PI;
        servoCommands[wheel].radius = sqrt(pow(Chassis[wheel].x,2)+pow(Chassis[wheel].y,2));
      }

      // When turning in place, one side wheels turn opposite the other. Change direction of
      // this inequality comparison to reverse turn-in-place direction relative to joystick.
      if (Chassis[wheel].x > 0)
      {
        servoCommands[wheel].radius *= -1;
      }
    }

    speedFromRadius(steering);
  }
  else
  {
    // Choose a reference wheel and, from there, calculate turn center X
    if (steering == 0)
    {
      referenceWheel = -1;
      turnCenterX = 0;
    }
    else
    {
      if (steering > 0)
      {
        referenceWheel = FRONT_RIGHT;
      }
      else
      {
        referenceWheel = FRONT_LEFT;
      }

      servoCommands[referenceWheel].angle = maxSteering * steering / 100.0;

      inRadians = servoCommands[referenceWheel].angle * M_PI / 180.0;
      turnCenterX = Chassis[referenceWheel].x + (Chassis[referenceWheel].y / tan(inRadians));

      // Store length of hypotenuse for later speed calculation
      servoCommands[referenceWheel].radius = abs(Chassis[referenceWheel].y / sin(inRadians));
    }

    // Calculate other wheel angles based on turn center X
    for (wheel = 0; wheel < 6; wheel++)
    {
      if (turnCenterX == 0)
      {
        servoCommands[wheel].angle = 0;
        servoCommands[wheel].speed = velocity;
      }
      else if (wheel != referenceWheel)
      {
        // The X-axis distance between this wheel and turn center
        float wheelToCenter = turnCenterX - Chassis[wheel].x;

        if (Chassis[wheel].steerServoId != -1)
        {
          // Calculate wheel angle, atan returns radians.
          inRadians = atan(Chassis[wheel].y/wheelToCenter);

          // Convert to degrees and store command
          servoCommands[wheel].angle = inRadians*180.0/M_PI;

          // Store hypotenuse (turning radius) in speed for later calculation.
          servoCommands[wheel].radius = abs(Chassis[wheel].y/sin(inRadians));
        }
        else
        {
          // No steering servo, but good to blank out data anyway.
          servoCommands[wheel].angle = 0;

          // This wheel lacks steering servo. This should only happen on the wheels
          // aligned with turn axis. (wheel coordinate y of zero.) So their turning
          // radius ("hypotenuse" in all the other trig calculations) becomes their
          // X relative to turn center.
          servoCommands[wheel].radius = abs(wheelToCenter);
        }
      }
    }

    if (abs(turnCenterX) > 0)
    {
      speedFromRadius(velocity);
    }
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
