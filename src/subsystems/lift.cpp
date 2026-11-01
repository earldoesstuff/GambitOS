#include "main.h"
#include "subsystems/lift.hpp"
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <chrono>
#include <thread>


pros::Controller master(pros::E_CONTROLLER_MASTER);

namespace lift {
/*--------------------------DECLARATIONS---------------------------*/  
    pros::MotorGroup motors({11, -12});

    const double cascade_error = 0.075;

    lemlib::PID cascade_pid(10, 0.0, 0.0, 5, false); //creates lemlib constructor for cascade lift


/*-----------------CASCADE PID-----------------------*/
    void move(double target_pos, double timeout) {
        float error;
        uint32_t time = pros::millis();
        do {
            //make into a while loop so it runs until 
            float current_pos = lift::motors.get_position(); //creates a variable that gets the motor position
            error = target_pos - current_pos; //determines error
            float power = cascade_pid.update(error); //gets motor power from pid algorithm
            lift::motors.move(power);
            pros::delay(10); //moves motor at required power //runs as a loop every 10 ms
        } while ((std::fabs(error) > cascade_error) && (pros::millis() - time <= timeout));
        lift::motors.move_voltage(0);
        lift::motors.brake();
    }

/*------------MANUAL CASCADE---------------*/
    void manual_cascade() {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            lift::motors.move_voltage(12000);

        }
        else if ((master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) && (motors.get_position() > 0)) {
            lift::motors.move_voltage(-6000);

        }
        else if (motors.get_position() < 10) {
            lift::motors.move_voltage(0);
        }
        else {
            lift::motors.move_voltage(0);
            lift::motors.brake();
        }      

    }

/*--------------INITIALIZE-----------------*/
    void init() {
        lift::motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
        lift::motors.set_zero_position_all(0);
        lift::motors.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    }

/*------------------LIFT AND CHAINBAR CONTROL---------------------*/
    void control() {
        while (true) {
            manual_cascade();
        pros::delay(10);
        }
    }

}