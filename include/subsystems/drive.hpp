#include "main.h"
#pragma once

namespace drive {
    extern pros::MotorGroup left_motor_group;
    extern pros::MotorGroup right_motor_group;
    extern lemlib::Chassis chassis;
    extern std::vector<int> left_motors;
    extern std::vector<int> right_motors;

    void init();
    void control();
}