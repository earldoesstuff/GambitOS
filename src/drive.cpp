#include "main.h"
#include "/Users/damoss/gambitOS/include/subsystems/drive.hpp"
#include "lemlib/api.hpp"



//lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);
// create the chassis
// input curve for throttle input during driver control
// drivetrain settings
// left motor group
pros::MotorGroup left_motor_group({12, 14, -15}, pros::MotorGears::blue);
// right motor group
pros::MotorGroup right_motor_group({17, 19, -20}, pros::MotorGears::blue);
// imu
pros::Imu imu(1);
//pros::Imu imu(1);
pros::Rotation horizontal_encoder(8); // 15 //

pros::Rotation vertical_encoder(3); //-14

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_325, -5.75);//THIS IS ODOM

lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_325, -2.5);

lemlib::Drivetrain drivetrain(&left_motor_group, &right_motor_group, 10, lemlib::Omniwheel::NEW_325,400, 2 
);
// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, nullptr, &horizontal_tracking_wheel, nullptr,&imu 
);


lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);
// lateral PID controller//learn ts 
lemlib::ControllerSettings lateral_controller(0, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(0, // proportional gain (kP) Keep increasing
                                              0, // integral gain (kI)
                                              0, // derivative gain (kD)
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
                        &throttle_curve, 
                        &steer_curve
);