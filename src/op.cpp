#include "myFunctions.h"
#include "main.h"
#include "pid.h"
#include "pros/misc.h"

void op(double drive_scale) {
    bool isClamped = false;
	bool isHooked = false;
	double x_pos, y_pos;
	int chain_count = 0;
	bool armIntake = false;
	bool detected = false;

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
		left.move((powf(2.718, -(drive_scale / 10)) + powf(2.718, (fabs(cntrl.get_analog(ANALOG_LEFT_Y)) - 127) / 10 * (1 - powf(2.718, -(drive_scale / 10))))) * cntrl.get_analog(ANALOG_LEFT_Y));
		right.move((powf(2.718, -(drive_scale / 10)) + powf(2.718, (fabs(cntrl.get_analog(ANALOG_RIGHT_Y)) - 127) / 10 * (1 - powf(2.718, -(drive_scale / 10))))) * cntrl.get_analog(ANALOG_RIGHT_Y));

		// intake and chain
		if (cntrl.get_digital(DIGITAL_L1)) {
			if (armIntake){
				if (armSensor.get()<60){
					detected = true;
				}
				if (detected){
					chain_count++;
					chain.move(-100);
					
				} else{
					chain.move(100);
				}
				if(chain_count > 400){
					chain_count = 0;
					detected = false;
				}
			} else{
				chain.move(100);
			}
			intake.move(127);
		} else if (cntrl.get_digital(DIGITAL_L2)) {
			intake.move(-127);
			chain.move(-100);
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
			wait(0.3);
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

		// arm unlock
		if (cntrl.get_digital(DIGITAL_A)) {
			arm.set_brake_mode(E_MOTOR_BRAKE_COAST);
		}

		// hook 
		if (cntrl.get_digital(DIGITAL_B)) {
			if (armIntake) {
				armIntake = false;
				cntrl.set_text(0, 0, "arm");
			} else {
				armIntake = true;
				cntrl.set_text(0, 0, "mogo");
			}
		}

		// chain reverse
		pros::delay(5);


	}
}