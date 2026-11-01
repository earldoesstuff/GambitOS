#pragma once
#include "main.h"

namespace lift {
    void move (double target_pos, double timeout);
    void init ();
    void control ();
    
    extern pros::MotorGroup cascade_motors;
}
extern pros::Controller master;