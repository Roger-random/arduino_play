#include "Arduino.h"
#include "joydrive.h"
#include "lewansoul.h"

#define STEERING_PIN 1
#define VELOCITY_PIN 0
#define BUTTON_PIN 2

JoyDrive jd(STEERING_PIN, VELOCITY_PIN,BUTTON_PIN);
LewanSoul lss(1);

void setup()
{
  lss.setup();
}

void loop()
{
  int steer = 0;
  
  steer = jd.getSteering(); // Returns value -100 to 100
  steer = steer*5; // Now from -500 to 500
  steer = steer+500; // Now from 0 to 1000
  lss.moveTo(30,steer);
  delay(250);
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
