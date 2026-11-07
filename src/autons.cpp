#include "main.h"
void one_pin_loader_side() {
    lift::startup();
    drive::chassis.setPose(-62, -7.5, 270);
    drive::chassis.moveToPoint(-55, -7.5, 1000, {.forwards = false});
    wrist::reset_position();
    lift::move(150, 1000);
    wrist::intake_motor.move_voltage(-4000);
    drive::chassis.waitUntilDone();
    drive::chassis.moveToPoint(-66, -7.5, 1000, { .maxSpeed=80, .minSpeed = 50,});//roller hit 1
    drive::chassis.waitUntilDone();
    pros::delay(125);
    drive::chassis.moveToPoint(-54, -7.5, 1000, {.forwards = false});
    drive::chassis.waitUntilDone();
    pros::delay(125);
    drive::chassis.moveToPoint(-66, -7.5, 1000, {.maxSpeed=80, .minSpeed = 50, }); //roller hit 2
    drive::chassis.waitUntilDone();
    pros::delay(125);
    drive::chassis.moveToPoint(-54, -7.5, 1500, {.forwards = false, .maxSpeed=60});
    drive::chassis.waitUntilDone();
    pros::delay(125);
    drive::chassis.turnToHeading(0, 500);
    drive::chassis.waitUntilDone();
    printf("X: %f, Y: %f, Theta: %f\n", drive::chassis.getPose().x, drive::chassis.getPose().y, drive::chassis.getPose().theta);
    drive::chassis.moveToPoint(-56, -24, 1500, {.forwards = false, .maxSpeed=60}); //lines up to goal
    lift::move(0, 500);
    wrist::angle(-300, 500);
    printf("X: %f, Y: %f, Theta: %f\n", drive::chassis.getPose().x, drive::chassis.getPose().y, drive::chassis.getPose().theta);
    drive::chassis.waitUntilDone();
    wrist::intake_motor.move_voltage(12000);
    pros::delay(500);
    wrist::angle(-100, 500);
    wrist::intake_motor.move_voltage(-6000);
    drive::chassis.moveToPoint(-52, -13, 1000);
    drive::chassis.waitUntilDone();
    printf("X: %f, Y: %f, Theta: %f\n", drive::chassis.getPose().x, drive::chassis.getPose().y, drive::chassis.getPose().theta); //move off point
    drive::chassis.turnToHeading(290, 500);
    drive::chassis.waitUntilDone();
    printf("X: %f, Y: %f, Theta: %f\n", drive::chassis.getPose().x, drive::chassis.getPose().y, drive::chassis.getPose().theta); //new heading
    drive::chassis.moveToPoint(-34, -31, 1000, { .forwards=false, .maxSpeed=40, .earlyExitRange=1,}); //tune ts
    drive::chassis.waitUntilDone();
    pros::delay(125);
    wrist::angle(-400, 750);
    drive::chassis.turnToHeading(300, 500);
    drive::chassis.turnToHeading(280, 500);
    pros::delay(250);
    wrist::angle(-300, 750);
    lift::move(150, 1000);
    drive::chassis.turnToHeading(65, 500);
    printf("X: %f, Y: %f, Theta: %f\n", drive::chassis.getPose().x, drive::chassis.getPose().y, drive::chassis.getPose().theta);//last point
    drive::chassis.moveToPoint(-60, -30, 1000, {.forwards=false});
}
/*trying to reduce error and slippage by avoiding tipping during rollers; lower */
void one_pin_blank_side() {
    lift::startup();
    drive::chassis.setPose(-62, -7.5, 270);
    drive::chassis.moveToPoint(-55, -7.5, 1000, {.forwards = false});
    wrist::reset_position();
    lift::move(150, 1000);
    wrist::intake_motor.move_voltage(-4000);
    drive::chassis.waitUntilDone();
    drive::chassis.moveToPoint(-66, -7.5, 1000, { .maxSpeed=80, .minSpeed = 50,});//roller hit 1
    drive::chassis.waitUntilDone();
    pros::delay(125);
    drive::chassis.moveToPoint(-54, -7.5, 1000, {.forwards = false});
    drive::chassis.waitUntilDone();
    pros::delay(125);
    drive::chassis.moveToPoint(-66, -7.5, 1000, {.maxSpeed=80, .minSpeed = 50, }); //roller hit 2
    drive::chassis.waitUntilDone();
    pros::delay(125);
    drive::chassis.moveToPoint(-51, -7.5, 1500, {.forwards = false, .maxSpeed=60});
    drive::chassis.waitUntilDone();
    pros::delay(125);
    drive::chassis.turnToHeading(180, 500);
    drive::chassis.waitUntilDone();
    printf("X: %f, Y: %f, Theta: %f\n", drive::chassis.getPose().x, drive::chassis.getPose().y, drive::chassis.getPose().theta);
    drive::chassis.moveToPoint(-53, 12, 1500, {.forwards = false, .maxSpeed=60}); //lines up to goal
    lift::move(0, 500);
    wrist::angle(-300, 500);
    printf("X: %f, Y: %f, Theta: %f\n", drive::chassis.getPose().x, drive::chassis.getPose().y, drive::chassis.getPose().theta);
    drive::chassis.waitUntilDone();
    wrist::intake_motor.move_voltage(12000);
    pros::delay(500);
    wrist::angle(-100, 500);
}

void skills() {
    lift::startup();
    drive::chassis.setPose(-62, -7.5, 270);
    drive::chassis.moveToPoint(-55, -7.5, 1000, {.forwards = false});
    wrist::reset_position();
    lift::move(150, 1000);
    wrist::intake_motor.move_voltage(-4000);
    drive::chassis.waitUntilDone();
    drive::chassis.moveToPoint(-66, -7.5, 1000, { .maxSpeed=80, .minSpeed = 50,});//roller hit 1
    drive::chassis.waitUntilDone();
    pros::delay(125);
    drive::chassis.moveToPoint(-54, -7.5, 1000, {.forwards = false});
    drive::chassis.waitUntilDone();
    pros::delay(125);
    drive::chassis.moveToPoint(-66, -7.5, 1000, {.maxSpeed=80, .minSpeed = 50, }); //roller hit 2
    drive::chassis.waitUntilDone();
    pros::delay(125);
    drive::chassis.moveToPoint(-54, -7.5, 1500, {.forwards = false, .maxSpeed=60});
    drive::chassis.waitUntilDone();
    pros::delay(125);
    drive::chassis.turnToHeading(0, 500);
    drive::chassis.waitUntilDone();
    printf("X: %f, Y: %f, Theta: %f\n", drive::chassis.getPose().x, drive::chassis.getPose().y, drive::chassis.getPose().theta);
    drive::chassis.moveToPoint(-56, -24, 1500, {.forwards = false, .maxSpeed=60}); //lines up to goal
    lift::move(0, 500);
    wrist::angle(-300, 500);
    printf("X: %f, Y: %f, Theta: %f\n", drive::chassis.getPose().x, drive::chassis.getPose().y, drive::chassis.getPose().theta);
    drive::chassis.waitUntilDone();
    wrist::intake_motor.move_voltage(12000);
    pros::delay(500);
    wrist::angle(-100, 500);
    drive::chassis.moveToPoint(-56, 0, 1500);
    drive::chassis.turnToHeading(0, 1500);
    drive::chassis.moveToPoint(-5
    , 0, 4000);
}