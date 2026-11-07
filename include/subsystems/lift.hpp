#pragma once
#include "main.h"

namespace lift {
    void move (double target_pos, double timeout);
    void init ();
    void control ();
    void startup();
    extern pros::MotorGroup motors;
}
extern pros::Controller master;