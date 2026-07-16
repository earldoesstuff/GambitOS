#include "main.h"
//#include ""
//#include "static"



ASSET(example_txt);


void disabled() {}
void initialize() {
    //start_autoselector();
    lift::init();
    drive::init();


}


void autonomous() {

}


void opcontrol() {
    pros::Task lift_task(lift::control, TASK_PRIORITY_DEFAULT + 2);
    pros::Task drive_task(drive::control, TASK_PRIORITY_DEFAULT + 3);
    while (true) {
        
        pros::delay(20);
    }
   
}

