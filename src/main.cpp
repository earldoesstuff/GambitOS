#include "main.h"



ASSET(example_txt);


void disabled() {}
void initialize() {
    //start_autoselector();
    lift::init();
    drive::init();


}


void autonomous() {
    pros::delay(500);
    lift::auton_cascade(1.0, 500);
}


void opcontrol() {
    pros::Task lift_task(lift::control, TASK_PRIORITY_DEFAULT + 4);
    pros::Task drive_task(drive::control, TASK_PRIORITY_DEFAULT + 3);
    while (true) {
        pros::delay(20);
    }
   
}

