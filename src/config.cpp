#include "config.h"
#include "pros/rotation.h"

using namespace pros;

Controller cntrl(CONTROLLER_MASTER);

Motor chain(17, MotorGears::blue, MotorEncoderUnits::rotations);
Motor intake(20, MotorGears::green, MotorEncoderUnits::rotations);
Motor arm(-2, MotorGears::red, MotorEncoderUnits::rotations);
MotorGroup left({-16, -15, -14}, MotorGears::blue, MotorEncoderUnits::rotations);
MotorGroup right({13, 12, 11}, MotorGears::blue, MotorEncoderUnits::rotations);
adi::DigitalOut clamp('H');
Imu inertial(14);
adi::Port hook({15, 'h'}, E_ADI_DIGITAL_OUT);
Rotation hwheel(7);
Rotation vwheel(8);
Distance armSensor(15);

double dead_wheel_radius = 2.0 / 2; // inches