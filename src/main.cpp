#include <cmath>
#include <tuple>

#include "myFunctions.h"
#include "op.h"
#include "config.h"
#include "tracking.hpp"
#include "pid.h"
#include "auton.hpp"
#include "controller.hpp"

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}
//lebron james

void initialize() {
	pros::lcd::initialize();
	resetEncoder();
	hwheel.set_position(0);
	vwheel.set_position(0);
	hwheel.set_data_rate(5);
	vwheel.set_data_rate(5);
	inertial.tare();
}


void disabled() {}


void competition_initialize() {
	initialize();
}


void autonomous() {
	
	ringRushBlue();
	// ringRushRed();
	// soloWP();
	// mogoRush();
}


void opcontrol() {
	op(0);
}