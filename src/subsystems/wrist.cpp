#include "main.h"
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <thread>

namespace wrist {
/*--------------------------DECLARATIONS---------------------------*/  
    pros::Motor angle_motor({-19}); //wrist motor declaration
    pros::Motor intake_motor({18});
    const double wrist_error = 1.0;
    bool claw_intaking = true;
    lemlib::PID wrist_pid(1.5, 0.0, 2.5, 5, false); //creates lemlib constructor for wrist pid

/*---------------WRIST PID--------------------*/
    void angle(double target_pos, double timeout) {
        float error;
        uint32_t time = pros::millis();
        do {
            float current_pos = angle_motor.get_position();
            error = target_pos - current_pos;
            float power = wrist_pid.update(error);
            angle_motor.move(power);
            pros::delay(10);
        } while ((std::fabs(error) > wrist_error) && (pros::millis() - time <= timeout));
        angle_motor.move_voltage(0);
        angle_motor.brake();
    }

/*-------INTAKE CONTROL------*/
    void claw_intake() {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake_motor.move_voltage(12000);
        }
        else if (claw_intaking == true) {
            intake_motor.move_voltage(-4000);
        }
        else {
            intake_motor.move_voltage(0);
        }
        
    }
    

    void reset_position() {
            angle_motor.move_voltage(12000);
            pros::delay(500);
            angle_motor.set_zero_position(0);
            angle_motor.move_voltage(0);
            angle_motor.brake();
    }
    void manual_control() {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            if (angle_motor.get_position() < -120) { //if position is lower than 120
                wrist::angle(-100, 1500); //move to higher position
            }
            else {
                wrist::angle(-360,500);
                wrist::angle(-300, 500);
            }
        }
        
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            reset_position();
        }
    }
/*--------------INITIALIZE-----------------*/
    void init() {
        angle_motor.set_brake_mode(MOTOR_BRAKE_HOLD);
        angle_motor.set_zero_position(0);
        angle_motor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    }

/*------------------WRIST CONTROL---------------------*/
    void control() {
        while (true) {
            claw_intake();
            manual_control();
            if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            claw_intaking = !claw_intaking;
            }
        pros::delay(10);
        }
    }

}