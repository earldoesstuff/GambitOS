#include "main.h"
#include "subsystems/lift.hpp"

namespace lift {
    
    pros::MotorGroup cascade_motors({9, -10});
    pros::MotorGroup chainbar_motors({-5,6});


    pros::adi::Pneumatics clamp_piston('c', false);
    pros::adi::DigitalIn  cascade_limit('b'); 

    cascade_mode_e_t cascade_mode = CASCADE_MANUAL;

    lemlib::PID cascade_pid(5, 0.01, 20, 5, false); 

    void auton_cascade(double target_pos) {
        float current_pos = cascade_motors.get_position();
        float error = target_pos - current_pos;
        float power = cascade_pid.update(error);
        cascade_motors.move(power);
        pros::delay(10);
    }


    void manual_cascade() {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            cascade_motors.move_voltage(12000);
        }
        else if ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) && (cascade_limit.get_value() == 0)) {
            cascade_motors.move_voltage(-12000);
        }
        else {
            cascade_motors.move_voltage(0);
            cascade_motors.brake();
        }
        pros::delay(10);
    }

    void init() {
        cascade_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
        chainbar_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
    }

    void control() {
        while (true) {
            switch (cascade_mode) {
                case 0:
                    manual_cascade();
                    break;
                case 1:
                    //calculate from lookup table here
                    double target_pos;
                    auton_cascade(target_pos);
                    break;
                default:
                    printf("dumbass\n");
                    break;
            }

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                chainbar_motors.move_voltage(-6000);
            }
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                chainbar_motors.move_voltage(6000);
            }
            else {
                chainbar_motors.move_voltage(0);
                chainbar_motors.brake();
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                clamp_piston.toggle();
            }
        }
    }

}