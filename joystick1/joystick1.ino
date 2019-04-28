#include "Arduino.h"
#include "joydrive.h"

#define STEERING_PIN 1
#define VELOCITY_PIN 0
#define BUTTON_PIN 2

JoyDrive jd(STEERING_PIN, VELOCITY_PIN,BUTTON_PIN);

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
