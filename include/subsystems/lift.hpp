#pragma once
#include "api.h"

// 1. Declare the update function so main.cpp can run it in its loop
void updateLift();

// 2. Share the global tracking variable 
extern bool isQuickDropping;

// 3. Share the motor and sensor objects (useful if you need them in autonomous.cpp)
extern pros::Motor cascadeLeft;
extern pros::Motor cascadeRight;
extern pros::adi::DigitalIn limitSwitch;