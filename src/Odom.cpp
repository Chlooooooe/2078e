#include "Odom.hpp"
#include "myFunctions.h"

double abs_x, abs_y;
double curr_h, curr_v, curr_or;
double prev_h, prev_v, prev_or;
double delta_h, delta_v, delta_or;

Odom::Odom(double absolute_x, double absolute_y, double start_angle) {
    abs_x = absolute_x;
    abs_y = absolute_y;
    prev_or = start_angle;
}

void Odom::update_offsets() {
    
    // update current values
    curr_h = hValue();
    curr_v = vValue();
    curr_or = getAngle();

    // update changes
    delta_h = curr_h - prev_h;
    delta_v = curr_v - prev_v;
    delta_or = curr_or - prev_or;

    // calculate local offset since previous (subtract turn movement)
    double offset_local_h, offset_local_v;
    if (delta_or == 0.0) {
        offset_local_h = delta_h;
        offset_local_v = delta_v;
    } else {
        offset_local_h = 2 * sinf(delta_or / 2) * ((delta_h / delta_or) + 0);
        offset_local_v = 2 * sinf(delta_or / 2) * ((delta_v / delta_or) - 0);
    }

    // calculate average field arc angle
    double or_avg = curr_or - (delta_or / 2);

    // calculate global offset
    double offset_global_x = (offset_local_v * sinf(or_avg)) + (offset_local_h * cos(or_avg));
    double offset_global_y = (offset_local_v * cos(or_avg)) - (offset_local_h * sin(or_avg));

    // update field positions
    abs_x += offset_global_x;
    abs_y += offset_global_y;

}

void Odom::update_previous() {
    prev_h = curr_h;
    prev_v = curr_v;
    prev_or = curr_or;
}

double Odom::getX() {
    return abs_x;
}

double Odom::getY() {
    return abs_y;
}