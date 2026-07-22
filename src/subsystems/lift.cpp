#include "main.h"
#include "subsystems/lift.hpp"
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <chrono>
#include <thread>

/*
initial/split
pickup height 0.18/0
lower pickup height 0
first cup scoring height 1.6/0.5 (front)
second cup scoring hight 0.3/0.02
third cup scoring height 0.95
*/
pros::Controller controller(pros::E_CONTROLLER_MASTER);

namespace lift {
/*--------------------------DECLARATIONS---------------------------*/  
    pros::MotorGroup cascade_motors({9, -10});
    pros::MotorGroup chainbar_motors({5,-6}); //cascade and chainbar motor declarations
    pros::adi::Pneumatics clamp_piston('c', false); //clamp piston declaration
    pros::adi::DigitalIn  cascade_limit('b');  //cascade limit switch declaration

    cascade_mode_e_t cascade_mode = CASCADE_MANUAL; //creates enum for manual/autonomous cascade pid
    bool cascade_mode_flag = false;
    bool was_moving = false;
    const std::vector <double> PIN_HEIGHTS = {0.1, 1.0, 2.0};
    const double cascade_error = 0.075;
    const double chainbar_error = 4.0;
//between 37695 and 3700 - 4075/4076
    lemlib::PID cascade_pid(3700, 0.0, 4075, 5, false); //creates lemlib constructor for cascade lift
    lemlib::PID chainbar_pid(6.75, 0.0, 25.0, 5, false); //creates lemlib constructor for chainbar
//between 6.5 and 6.75 - 0.5 and 

/*-----------------CASCADE PID-----------------------*/
    void auton_cascade(double target_pos) {
        float error;
        /*do {
            float current_pos = cascade_motors.get_position();
            error = target_pos - current_pos;
            cascade_motors.move_voltage(12000);
            pros::delay(10);
        } while (std::fabs(error) > cascade_error);*/
        do {
            //make into a while loop so it runs until 
            float current_pos = cascade_motors.get_position(); //creates a variable that gets the motor position
            error = target_pos - current_pos; //determines error
            float power = cascade_pid.update(error); //gets motor power from pid algorithm
            cascade_motors.move(power);
            pros::delay(10); //moves motor at required power //runs as a loop every 10 ms
        } while (std::fabs(error) > cascade_error);
    }

/*------------MANUAL CASCADE---------------*/
    void manual_cascade() {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            cascade_motors.move_voltage(12000);

        }
        else if ((controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) && (cascade_limit.get_value() == 0)) {
            cascade_motors.move_voltage(-12000);

        }
        else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            printf("started\n");
            double curr_height = cascade_motors.get_position();
            double target_pos = closest_height(PIN_HEIGHTS, curr_height);
            auton_cascade(target_pos);
            printf("running\n");
        }
        else {
            cascade_motors.move_voltage(0);
            cascade_motors.brake();
                    
            
            /*if (was_moving) {
                cascade_mode_flag = true;
                was_moving = false;
            }*/
        }
    }

/*---------------CHAINBAR PID--------------------*/
    void auton_chainbar(double target_pos) {
        float error;
        do {
            float current_pos = chainbar_motors.get_position();
            error = target_pos - current_pos;
            float power = chainbar_pid.update(error);
            chainbar_motors.move(power);
            pros::delay(10);
        } while (std::fabs(error) > chainbar_error);
        chainbar_motors.move_voltage(0);
        chainbar_motors.brake();
    }

/*-------------CLOSEST HEIGHT LOOKUP TABLE--------------*/

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


    void cascade_reset() {
        do {
         cascade_motors.move_voltage(-12000);   
        } while ((cascade_limit.get_value()) || (cascade_motors.get_position() > 0));
        cascade_motors.move_voltage(0);
        cascade_motors.brake();
    }

/*--------------INITIALIZE-----------------*/
    void init() {
        cascade_motors.set_brake_mode(MOTOR_BRAKE_BRAKE);
        cascade_motors.set_zero_position_all(0);
        cascade_motors.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
        chainbar_motors.set_brake_mode(MOTOR_BRAKE_HOLD);
        chainbar_motors.set_zero_position_all(0);
        chainbar_motors.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
    }

/*------------------LIFT AND CHAINBAR CONTROL---------------------*/
    void control() {
        while (true) {

            manual_cascade();

            
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            printf("%f\n", cascade_motors.get_position());
            printf("%f\n", chainbar_motors.get_position());
            }

            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) { //score macro 2
                auton_cascade(cascade_motors.get_position() - 0.2);
            }
            else {
                chainbar_motors.move_voltage(0);
                chainbar_motors.brake();
            }



            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {

            }
            /*---------FIRST FRONT MACRO---------*/
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) { //first front
                auton_chainbar(120);
                auton_cascade(1.6);
            }
            /*--------QUICKDROP MACRO--------*/
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) { //quick drop macro
                clamp_piston.retract();
                auton_cascade((cascade_motors.get_position() + 0.3));
                auton_chainbar(0);
                auton_cascade(0.25);
            }

            /*--------PICKUP MACRO----------*/
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) { //pickup macro
                cascade_reset();
                printf("1\n");
                clamp_piston.extend();
                printf("2\n");
                auton_cascade(0.35);
                printf("done\n");
            }
            /*else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
                clamp_piston.retract();
            }*/

            /*------LIMIT SWITCH------*/
            if (cascade_limit.get_new_press()) {
                cascade_motors.set_zero_position(0);
            }
        pros::delay(10);
        }
    }

}