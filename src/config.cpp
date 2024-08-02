#include "config.h"
#include "pros/rotation.h"

using namespace pros;

Controller cntrl(CONTROLLER_MASTER);

Motor chain(1, MotorGears::green, MotorEncoderUnits::rotations);
Motor intake(10, MotorGears::green, MotorEncoderUnits::rotations);
Motor arm(-2, MotorGears::red, MotorEncoderUnits::rotations);
MotorGroup left({-11, -12, -13}, MotorGears::blue, MotorEncoderUnits::rotations);
MotorGroup right({18, 19, 20}, MotorGears::blue, MotorEncoderUnits::rotations);
adi::DigitalOut clamp('H');
Imu inertial(14);
adi::Port hook({15, 'h'}, E_ADI_DIGITAL_OUT);
Rotation hwheel(7);
Rotation vwheel(8);
Distance armSensor(15);

double dead_wheel_radius = 2.0 / 2; // inches