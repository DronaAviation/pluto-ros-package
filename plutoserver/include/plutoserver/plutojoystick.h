#ifndef PLUTOJOYSTICK_H
#define PLUTOJOYSTICK_H

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "plutodrone/PlutoMsg.h"

const int ARM_BTN = 4;
const int TAKE_OFF_BTN = 3;
const int LAND_BTN = 1;
const int TRIM_ROLL_AXES = 4;
const int TRIM_PITCH_AXES = 5;
const int TRIM_SAVE_BTN = 5;
const int RC_ROLL_AXES = 2;
const int RC_PITCH_AXES = 3;
const int RC_YAW_AXES = 0;
const int RC_THROTTLE_AXES = 1;
const int AUTO_PILOT_BTN = 7;


float mapping(float x, float inMin, float inMax, float outMin, float outMax);
void arm();
void disarm();
void box_arm();
void take_off();
void land();

#endif
