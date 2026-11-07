#include "main.h"
#include "subsystems/drive.hpp"

#define LEFT_MOTORS {-8, 9, 10}
#define RIGHT_MOTORS {-5, -6, 7} //flipped

namespace drive {

    std::vector<int> left_motors = LEFT_MOTORS;
    std::vector<int> right_motors = RIGHT_MOTORS;

    pros::MotorGroup left_motor_group(LEFT_MOTORS, pros::MotorGears::blue);
    pros::MotorGroup right_motor_group(RIGHT_MOTORS, pros::MotorGears::blue);

    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    lemlib::Drivetrain drivetrain(&right_motor_group, &left_motor_group, 10.5, lemlib::Omniwheel::NEW_275, 450, 8); //switched right and left
    
    pros::Imu imu(20);

    lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

    // lateral PID controller
    lemlib::ControllerSettings lateral_controller(
                                                9.5, // proportional gain (kP) //temp change from 11.5
                                                0, // integral gain (kI)
                                                29, // derivative gain (kD)
                                                3, // anti windup
                                                1, // small error range, in inches
                                                100, // small error range timeout, in milliseconds
                                                3, // large error range, in inches
                                                0, // large error range timeout, in milliseconds
                                                0 // maximum acceleration (slew)
    );

    // angular PID controller
    lemlib::ControllerSettings angular_controller(
                                                3, // proportional gain (kP)1.5 2
                                                0, // integral gain (kI)
                                                27, // derivative gain (kD)
                                                3, // anti windup
                                                1, // small error range, in degrees
                                                100, // small error range timeout, in milliseconds
                                                3, // large error range, in degrees
                                                500, // large error range timeout, in milliseconds
                                                0 // maximum acceleration (slew)
    );

    // input curve for steer input during driver control
    lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                    10, // minimum output where drivetrain will move out of 127
                                    1.019 // expo curve gain
    );

    lemlib::Chassis chassis(drivetrain,
                            lateral_controller,
                            angular_controller,
                            sensors,
                            nullptr, 
                            &steer_curve
    );

    void init() {
        chassis.calibrate();
        chassis.setBrakeMode(MOTOR_BRAKE_COAST);
    }

    void control() {
        chassis.setBrakeMode(MOTOR_BRAKE_COAST);
        while (true) {
            int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            chassis.arcade(leftY, rightX, true, 0.7);
            //chassis.arcade((-1 * controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)), controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), true, 0.7);
        pros::delay(10);
        }
    }
}