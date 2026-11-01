#include "main.h"

namespace intake {
    pros::Motor intake_motor({-15});

    void loading_state() {
        lift::move(250, 1000);
        wrist::angle(-620, 500);
    }

    void manual_intake() {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake::intake_motor.move_voltage(12000);
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake::intake_motor.move_voltage(-12000);
        }
        else {
            intake::intake_motor.move_voltage(0);
        }
    }

    void control() {
        while (true) {
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
                intake::loading_state();
            }
            manual_intake();
        pros::delay(10);
        }
    }

}