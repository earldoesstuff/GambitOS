#include "main.h"


pros::Motor cascadeLeft(13);
pros::Motor cascadeRight(-7);
pros::adi::DigitalOut piston('a', 'false');

// Motor and Sensor Declarations
// Adjust the port numbers and letters to match your physical robot

pros::adi::DigitalIn limitSwitch('B'); 

// Variable to keep track of the automated quick drop state

// Tracking variables
bool isQuickDropping = false;
bool pistonState = false;       // false = retracted, true = extended
uint32_t liftStartTime = 0;     // Keeps track of when the lift started moving
bool isAutoLifting = false;     // True while the 1-second lift is running

void updateLift() {
    bool isLimitSwitchHit = limitSwitch.get_value();

   
    // Safety check: kill down movements if limit switch is hit
    if (isLimitSwitchHit && (isQuickDropping || controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))) {
        isQuickDropping = false; 
    }

    // Trigger Quick Drop (Y Button)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y) && !isLimitSwitchHit) {
        isQuickDropping = true; 
        isAutoLifting = false; // Cancel any active auto-lifting
    }

    // Trigger Piston Toggle & Auto-Lift Timer (R1 Button)
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
        isQuickDropping = false; 
        pistonState = !pistonState; 
        piston.set_value(pistonState);

        if (pistonState == true) {
            liftStartTime = pros::millis(); 
            isAutoLifting = true;
        } else {
            isAutoLifting = false; // Retracting immediately cancels the lift timer
        }
    }

    // ==========================================================
    // 2. MOTOR PRIORITY STACK (Only determines motor voltages)
    // ==========================================================
    
    // Check if the 1-second timer is currently active
    if (isAutoLifting) {
        if (pros::millis() - liftStartTime < 1000) {
            cascadeLeft.move_voltage(12000);
            cascadeRight.move_voltage(12000);
        } else {
            isAutoLifting = false; // Timer expired! Turn off the auto-lift flag
        }
    }
    
    // If the 1-second lift is NOT active, check manual buttons and quick-drop
    if (!isAutoLifting) {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            // Manual Up: Allows you to hold L1 OR R1 to keep moving up after the timer ends!
            isQuickDropping = false;
            cascadeLeft.move_voltage(12000);
            cascadeRight.move_voltage(12000);
        } 
        else if (isQuickDropping) {
            // Automated Quick Drop
            cascadeLeft.move_voltage(-12000);
            cascadeRight.move_voltage(-12000);
        } 
        else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            // Manual Down
            if (!isLimitSwitchHit) {
                cascadeLeft.move_voltage(-12000);
                cascadeRight.move_voltage(-12000);
            } else {
                cascadeLeft.brake();
                cascadeRight.brake();
            }
        } 
        else {
            // Default Rest State: No buttons pressed, no timers active
            cascadeLeft.move_voltage(0);
            cascadeRight.move_voltage(0);
        }
    }
}