#pragma once
#include "main.h"

namespace wrist {
    void angle (double target_pos, double timeout);
    void init ();
    void control ();
    void reset_position();
    void loading_position();

    extern pros::Motor angle_motor;
    extern pros::Motor intaking_wrist_motor;

}