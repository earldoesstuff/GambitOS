#include "main.h"
#include "subsystems/lift.hpp"
#include <iostream>
#include <algorithm>
#include <cstdlib>



pros::Controller controller(pros::E_CONTROLLER_MASTER);

namespace lift {
/*--------------------------DECLARATIONS---------------------------*/  
    pros::MotorGroup cascade_motors({9, -10});
    pros::MotorGroup chainbar_motors({-5,6}); //cascade and chainbar motor declarations

    pros::adi::Pneumatics clamp_piston('c', false); //clamp piston declaration
    pros::adi::DigitalIn  cascade_limit('b');  //cascade limit switch declaration

    cascade_mode_e_t cascade_mode = CASCADE_MANUAL; //creates enum for manual/autonomous cascade pid
    bool cascade_mode_flag = false;
    bool was_moving = false;
    const std::vector <double> PIN_HEIGHTS = {10.0, 20.0, 30.0};
    const double cascade_error = 0.25;
    const double chainbar_error = 0.25;

    lemlib::PID cascade_pid(5, 0.01, 20, 5, false); //creates lemlib constructor for cascade lift
    lemlib::PID chainbar_pid(5 , 0.01, 20, 5, false); //creates lemlib constructor for chainbar

/*-----------------CASCADE PID-----------------------*/
    void auton_cascade(double target_pos) {
        float error;
        do {
            //make into a while loop so it runs until 
            float current_pos = cascade_motors.get_position(); //creates a variable that gets the motor position
            error = target_pos - current_pos; //determines error
            float power = cascade_pid.update(error); //gets motor power from pid algorithm
            cascade_motors.move(power); //moves motor at required power //runs as a loop every 10 ms
        } while (std::fabs(error) > cascade_error);
        cascade_mode = CASCADE_MANUAL;
    }

/*------------MANUAL CASCADE---------------*/
    void manual_cascade() {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            cascade_motors.move_voltage(12000);
            was_moving = true;
        }
        else if ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) && (cascade_limit.get_value() == 0)) {
            cascade_motors.move_voltage(-12000);
            was_moving = true;
        }
        else {
            cascade_motors.move_voltage(0);
            cascade_motors.brake();
            if (was_moving) {
                cascade_mode_flag = true;
                was_moving = false;
            }
        }
    }

/*---------------CHAINBAR PID--------------------*/
    void auton_chainbar(double target_pos) {
        float error;
        do {
            float current_pos = chainbar_motors.get_position();
            error = target_pos - current_pos;
            float power = cascade_pid.update(error);
            cascade_motors.move(power);
            pros::delay(10);
        } while (std::fabs(error) > chainbar_error);
    }

/*-------------LOOKUP TABLE--------------*/

    double closest_height(const std::vector<double>&heights, double curr_height) {
        auto next_state = std::lower_bound(heights.begin(), heights.end(), curr_height);
            
            if (next_state == heights.end()) {
                return heights.back();
            }
            if (next_state == heights.begin()) {
                return *next_state;
            }

            double higher = *next_state;
            double lower = *(next_state-1);
            if (curr_height - lower <= higher - curr_height) {
                return lower;
            }
            else {
                return higher;
            }
    }


/*--------------INITIALIZE-----------------*/
    void init() {
        cascade_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
        cascade_motors.set_zero_position_all(0);
        cascade_motors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
        chainbar_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
        chainbar_motors.set_zero_position_all(0);
        chainbar_motors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    }
/*--------------MACROS-----------------*/
    /*void quick_drop_macro() {
        cascade_mode = CASCADE_AUTO;
        lift::auton_cascade(10);
        lift::auton_chainbar(0);
        pros::delay(500);
        lift::clamp_piston.extend();
        pros::delay(500);
        auton_cascade(20);
        cascade_mode = CASCADE_MANUAL;
    }

    void pickup_macro() {
        cascade_mode = CASCADE_AUTO;
        auton_cascade(0);
        pros::delay(250);
        clamp_piston.extend();
        pros::delay(250);
        auton_cascade(10);
    }*/
/*------------------LIFT AND CHAINBAR CONTROL---------------------*/
    void control() {
        while (true) {
            /*if (cascade_mode_flag) {
                cascade_mode = CASCADE_AUTO;
                cascade_mode_flag = false;
            }*/

           if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            printf("%f\n", cascade_motors.get_position());
           }
            switch (cascade_mode) { //driver control only
                case CASCADE_MANUAL:
                    manual_cascade();
                    break;
                case CASCADE_AUTO: {
                    double curr_height = cascade_motors.get_position(0);
                    double target_pos = closest_height(PIN_HEIGHTS, curr_height);
                    //calculate from lookup table here
                    auton_cascade(target_pos);
                    printf("auto\n");
                    break;}
                default:
                    printf("dumbass\n");
                    break;
            }

            if (cascade_limit.get_value()) {
                cascade_motors.set_zero_position(0);
            }
        pros::delay(10);
        }
    }

}

/*-----------LOOKUP TABLES----------*/


//if 

/*if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
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
            
*/

/* 
input x, output y

if x is greater than or equal to (cup lowest point + offset) and less than (cup highest point + offset) {
set enum
}
*/
