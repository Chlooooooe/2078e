#include "main.h"
#include "config.h"
#include "pid.cpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
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
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	initialize();
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	moveP(1, 300, 10);
	wait(3);
	turnP(1, 100, 90);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	bool isClamped = false;
	double drivescale = 4.8;

	while (true) {

		// print motor temps to brain (Celsius)
		pros::lcd::print(0, "left temp: %f", left.get_temperature_all());
		pros::lcd::print(1, "right temp: %f", right.get_temperature_all());
		pros::lcd::print(2, "intake temp: %f", intake.get_temperature());
		pros::lcd::print(3, "chain temp: %f", chain.get_temperature());
		// print motor wattage to brain (Watts)
		pros::lcd::print(4, "left power: %f", left.get_power_all());
		pros::lcd::print(5, "right power: %f", right.get_power_all());
		pros::lcd::print(6, "intake power: %f", intake.get_power());
		pros::lcd::print(7, "chain power: %f", chain.get_power());

		// linear drive
		// left.move(cntrl.get_analog(ANALOG_LEFT_Y));
		// right.move(cntrl.get_analog(ANALOG_RIGHT_Y));

		// curve drive
		left.move((powf(2.718, -(drivescale / 10)) + powf(2.718, (fabs(cntrl.get_analog(ANALOG_LEFT_Y)) - 127) / 10 * (1 - powf(2.718, -(drivescale / 10))))) * cntrl.get_analog(ANALOG_LEFT_Y));
		right.move((powf(2.718, -(drivescale / 10)) + powf(2.718, (fabs(cntrl.get_analog(ANALOG_RIGHT_Y)) - 127) / 10 * (1 - powf(2.718, -(drivescale / 10))))) * cntrl.get_analog(ANALOG_RIGHT_Y));

		// intake and chain
		if (cntrl.get_digital(DIGITAL_L1)) {
			intake.move(127);
			chain.move(127);
		} else if (cntrl.get_digital(DIGITAL_L2)) {
			intake.move(-127);
			chain.move(-127);
		} else {
			intake.move(0);
			chain.move(0);
		}
		
		// clamp
		if (cntrl.get_digital(DIGITAL_DOWN)) {
			if (isClamped) {
				clamp.set_value(0);
				isClamped = false;
			} else {
				clamp.set_value(1);
				isClamped = true;
			}
			pros::delay(150);
		}

		// arm
		if (cntrl.get_digital(DIGITAL_R1)) {
			arm.move(127);
		} else if (cntrl.get_digital(DIGITAL_R2)) {
			arm.move(-127);
		} else {
			arm.move(0);
			arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);
		}

		// hook b

	}
}