#pragma once
#include "main.h"

namespace lift {
    void auton_cascade (double target_pos, double timeout);
    void auton_chainbar (double target_pos, double timeout, double max_speed);
    double closest_height(const std::vector<double>&heights, double curr_height);
    void init ();
    void control ();

    extern pros::MotorGroup cascade_motors;
    extern pros::MotorGroup chainbar_motors;
    extern pros::adi::Pneumatics clamp_piston;
    extern pros::adi::DigitalIn cascade_limit;

}