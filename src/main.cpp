#include "main.h"



void disabled() {}
void initialize() {
    //start_autoselector();
    
    drive::init();
    lift::init();
    wrist::init();
}

/*
CONTROL SCHEME:
Left bumpers: Cascade Operation
Right bumpers: intake/claw
B - Loading State/Scoring Toggle
Y - Ground Pickup/Scoring Toggle
A - Lift Hardstop Override
Up - Reset Wrist Position
Left - Reset Cascade Position
*/

void autonomous() {
    skills();
    //one_pin_loader_side();
    //one_pin_blank_side();
}


void opcontrol() {
    pros::Task drive_task(drive::control, TASK_PRIORITY_DEFAULT + 4);
    pros::Task lift_task(lift::control, TASK_PRIORITY_DEFAULT + 3);
    pros::Task wrist_task(wrist::control, TASK_PRIORITY_DEFAULT + 2);
    pros::Task intake_task(intake::control, TASK_PRIORITY_DEFAULT + 1); 
    wrist::reset_position();
    drive::chassis.setPose(-63, -7.5, 270);
    //lift::motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    while (true) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        printf("X: %f, Y: %f, Theta: %f\n", drive::chassis.getPose().x, drive::chassis.getPose().y, drive::chassis.getPose().theta);
        }
        pros::delay(20);
    }
   
}

