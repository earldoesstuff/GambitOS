#include "main.h"



ASSET(example_txt);


void disabled() {}
void initialize() {
    //start_autoselector();
    lift::init();
    drive::init();
    wrist::init();


}


void autonomous() {
    drive::chassis.setPose(0, 0, 0);
    //drive::chassis.turnToHeading(90, 100000);
    drive::chassis.moveToPoint(0, -10, 1000, {.forwards=false});
    drive::chassis.turnToHeading(90, 1000);
    drive::chassis.moveToPoint(-10, -10, 1500, {.forwards=false});
}


void opcontrol() {
    pros::Task lift_task(lift::control, TASK_PRIORITY_DEFAULT + 3);
    pros::Task drive_task(drive::control, TASK_PRIORITY_DEFAULT + 4);
    pros::Task wrist_task(wrist::control, TASK_PRIORITY_DEFAULT + 2);
    pros::Task intake_task(intake::control, TASK_PRIORITY_DEFAULT + 1);
    wrist::reset_position();
    while (true) {
        pros::delay(20);
    }
   
}

