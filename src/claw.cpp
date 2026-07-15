#include "main.h"

//4 6




pros::Motor claw(4);   // Change 10 to your claw motor port
pros::Motor claw2(6);



void clawFN(void*) {
    while (true) {

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            claw.move_voltage(10000);
            claw2.move_voltage(-10000);
            
        }

        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            claw.move_voltage(-10000);
            claw2.move_voltage(10000);
        }

        else {
            claw.move_voltage(0);
            claw2.move_voltage(0);
            claw.brake();
            claw2.brake();
        }

        pros::delay(10);
    }
}

void startClawTask() {
    static pros::Task clawTask(clawFN);
}