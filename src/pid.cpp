#include <tuple>
#include <cmath>
#include "config.h"
#include "Odom.hpp"

std::tuple<double, double, double> PidCalc(double kp, double ki, double kd, double target, double time, double current, double prev_error, double integral) {
	double error = target - current;
	double wrapError = (error > 0) ? error - 360 : error + 360;
    error = (fabs(error) < fabs(wrapError)) ? error : wrapError;
	integral += error * time/1000;
	double derivative = (error - prev_error) *1000 / time;
	double output = kp * error + ki * integral + kd * derivative;
	prev_error = error;
	return std::make_tuple(output, error, integral);
}

int PidMoveTo(double targetPos, double kp, double ki, double kd, Odom odometry) { //Note this is only foward relative movement
	double threshold = 1;
	double threshold_speed =60;
	double currentPos = 0;
	int timeout = 1000;
	double prev_error = targetPos;
	double output, prev_integral = 0, count = 0;
	double initial_x = odometry.getX();
	double initial_y = odometry.getY();
	int time = 10;
	
	while (((fabs(prev_error) > threshold) || ((threshold_speed < (fabs(left.get_actual_velocity()) + fabs(right.get_actual_velocity()) )/2))) && (count < timeout)){
		odometry.update_offsets();

		currentPos = std::pow(std::pow((odometry.getX() - initial_x),2) + std::pow((odometry.getY() - initial_y),2),0.5);
		std::tie(output, prev_error, prev_integral) = PidCalc(kp, ki, kd, targetPos, time, currentPos, prev_error, prev_integral);
		
		left.move(output);
		right.move(output);

		odometry.update_previous();
		count += time;
		pros::delay(time);
	}
	return count;
}


int PidTurnTo(double targetHeading) {
	double kp,ki,kd;
	double threshold = 1;
	double threshold_speed =40;
	double currentHeading = inertial.get_heading();
	double timeout = 1000;
	double prev_error = targetHeading - currentHeading;
	double output, prev_integral = 0, count = 0;
	int time = 10;
	if (prev_error <30){
		kp = 1.56;
		ki = 0.0315;
		kd = 0.0075;
	}
	else{
		kp = 1.266;
		ki = 0.025;
		kd = 0.005;
	}
	while (((fabs(prev_error) > threshold) || ((threshold_speed < (fabs(left.get_actual_velocity()) + fabs(right.get_actual_velocity()) )/2))) && (count < timeout)){
		currentHeading = inertial.get_heading();
		std::tie(output, prev_error, prev_integral) = PidCalc(kp, ki, kd, targetHeading, time, currentHeading, prev_error, prev_integral);
		left.move(output);
		right.move(-output);
		count += time;
		pros::delay(time);
	}
	return count;
}

std::tuple<bool> PidTurnTo(double targetHeading, bool armIntake) {
	double kp,ki,kd;
	double threshold = 1;
	double threshold_speed =40;
	double currentHeading = inertial.get_heading();
	double timeout = 1000;
	double prev_error = targetHeading - currentHeading;
	double output, prev_integral = 0, count = 0;
	int time = 10;
	if (prev_error <30){
		kp = 1.56;
		ki = 0.0315;
		kd = 0.0075;
	}
	else{
		kp = 1.266;
		ki = 0.025;
		kd = 0.005;
	}
	while (((fabs(prev_error) > threshold) || ((threshold_speed < (fabs(left.get_actual_velocity()) + fabs(right.get_actual_velocity()) )/2))) && (count < timeout)){
		currentHeading = inertial.get_heading();
		std::tie(output, prev_error, prev_integral) = PidCalc(kp, ki, kd, targetHeading, time, currentHeading, prev_error, prev_integral);
		left.move(output);
		right.move(-output);
		count += time;
		pros::delay(time);
        if (armIntake){
            if (armSensor.get()<60){
                chain.move(0);
            }
		}
	}
	return std::make_tuple(armIntake);
}

int PidTurnLeftTo(double targetHeading) {
	double kp,ki,kd;
	kp = 2.724;
	//2.724
	double threshold = 1;
	double threshold_speed =40;
	double currentHeading = inertial.get_heading();
	double timeout = 1000;
	double prev_error = targetHeading - currentHeading;
	double output, prev_integral = 0, count = 0;
	int time = 10;
	if (prev_error <30){
		kp = 1.56;
		ki = 0.0315;
		kd = 0.0075;
	}
	else{
		kp = 1.266;
		ki = 0.025;
		kd = 0.005;
	}

	while (((fabs(prev_error) > threshold) || ((threshold_speed < (fabs(left.get_actual_velocity()) + fabs(right.get_actual_velocity()) )/2))) && (count < timeout)){
		currentHeading = inertial.get_heading();
		std::tie(output, prev_error, prev_integral) = PidCalc(kp, ki, kd, targetHeading, time, currentHeading, prev_error, prev_integral);
		left.move(output);
		count += time;
		pros::delay(time);
	}
	return count;
}

int PidTurnRightTo(double targetHeading) {
	double kp,ki,kd;
	double threshold = 1;
	double threshold_speed =40;
	double currentHeading = inertial.get_heading();
	double timeout = 1000;
	double prev_error = targetHeading - currentHeading;
	double output, prev_integral = 0, count = 0;
	int time = 10;
	if (prev_error <30){
		kp = 1.56;
		ki = 0.0315;
		kd = 0.0075;
	}
	else{
		kp = 1.266;
		ki = 0.025;
		kd = 0.005;
	}
	while (((fabs(prev_error) > threshold) || ((threshold_speed < (fabs(left.get_actual_velocity()) + fabs(right.get_actual_velocity()) )/2))) && (count < timeout)){
		currentHeading = inertial.get_heading();
		std::tie(output, prev_error, prev_integral) = PidCalc(kp, ki, kd, targetHeading, time, currentHeading, prev_error, prev_integral);
		right.move(-output);
		count += time;
		pros::delay(time);
	}
	return count;
}

void AutoPidTuning(double targetParam, double iterations, double rangeLower, double rangeHigher) {
	double inc = (rangeHigher - rangeLower) / (iterations - 1);
	double count, count1, lowestOutput = 0;
	double lowest = 100000;
	Odom odometry(0,0,0);
	pros::lcd::initialize;
	for (double i = rangeLower; i <= rangeHigher; i += inc) {
		// count = PidTurnLeftTo(targetParam, i, 0, 0);
		pros::delay(100);
		// count1 = PidTurnLeftTo(0, i, 0, 0);
		pros::delay(100);
		if (lowest > (count1 + count) / 2) {
			lowest = (count1 + count) / 2;
			lowestOutput = i;
		}
	}
	pros::lcd::print(0, "time: %f", lowest);
	pros::lcd::print(1, "value: %f", lowestOutput);

}