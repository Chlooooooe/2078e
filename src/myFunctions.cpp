#include "myFunctions.h"
#include "config.h"

using namespace pros;

void setdrive(double leftpower, double rightpower) {
    left.move(leftpower);
    right.move(rightpower);
}

void brakehold() {
    left.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    right.set_brake_mode(E_MOTOR_BRAKE_HOLD);
}

void brakecoast() {
    left.set_brake_mode(E_MOTOR_BRAKE_COAST);
    right.set_brake_mode(E_MOTOR_BRAKE_COAST);
}

double getLeftEncoder() {
    return left.get_position();
}

double getRightEncoder() {
    return right.get_position();
}

double getEncoderAvg() {
    return (getLeftEncoder() + getRightEncoder()) / 2;
}

void resetEncoder() {
    left.tare_position();
    right.tare_position();
}

void wait(double seconds) {
    pros::delay(seconds * 1000);
}

double getAngle() {
    return inertial.get_heading() * M_PI / 18000.0;
}

double hValue() {
    double degrees = hwheel.get_position() / 18000.0;
    return (degrees * M_PI) * dead_wheel_radius;
}

double vValue() {
    double degrees = vwheel.get_position() / 100.0;
    return (degrees * M_PI) * dead_wheel_radius;
}

void moveP(double direction, double speed, double dist) {
    resetEncoder();
    double speedP = speed/100;
    double targetVal = dist*70;
    double drifterror =0;
    double error = 1;
    double kP =100/targetVal;
    double Pval= speed;
    double finalPower = speed;
  
    while (abs(getEncoderAvg())<targetVal){
        error = targetVal - abs(getEncoderAvg());
        Pval = error*kP*speedP;
        finalPower = Pval*direction;
        setdrive(finalPower,finalPower);
        wait(0.02);
    }
}

void turnP(int dir, double speed, double dist){
    
    double targetVal = 0;
    double anglePower = speed;
    double driftError = (dist-(getAngle()));
    double kPDrift = speed/abs(driftError);
    while(abs(driftError)>10){
        driftError = (dist-(getAngle()));
        anglePower = driftError*kPDrift;
        setdrive(anglePower, anglePower);
        wait(0.02);
    }
    brakehold();
}

