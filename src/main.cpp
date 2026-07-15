#include "main.h"
#include "subsystems/drive.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "/Users/damoss/GambitOS/include/subsystems/drive.hpp"
#include "/Users/damoss/GambitOS/include/subsystems/lift.hpp"
#include "/Users/damoss/GambitOS/include/subsystems/claw.hpp"
#include "/Users/damoss/GambitOS/include/subsystems/pneumatics.hpp"
//#include ""
//#include "static"

pros::Controller controller(pros::E_CONTROLLER_MASTER);

ASSET(example_txt);


void disabled() {}
void initialize() {
    //start_autoselector();
     pros::lcd::initialize(); 
    chassis.calibrate(); 
	
    // 1. Set your motor brake modes early
    claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    claw2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    
    
}


void autonomous() {




    


    pros::delay(5);
}





void opcontrol() {
   chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  


    startClawTask();

	updateLift();



    while (true) {

        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        // move the robot
        chassis.arcade(rightX, leftY);

       


   
        pros::delay(10);
    }
   
}

