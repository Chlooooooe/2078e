#include "main.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"

using namespace pros;

extern Controller cntrl;

extern Motor intake;
extern Motor chain;
extern Motor arm;
extern MotorGroup left;
extern MotorGroup right;
extern adi::DigitalOut clamp;
extern Imu inertial;
extern adi::Port hook;
extern Rotation hwheel;
extern Rotation vwheel;
extern Distance armSensor;

extern double dead_wheel_radius;
