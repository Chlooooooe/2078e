#include "config.h"

void setdrive(double leftpower, double rightpower);
void brakehold();
void brakecoast();
double getLeftEncoder();
double getRightEncoder();
double getEncoderAvg();
void resetEncoder();
void wait(double seconds);
double getAngle();
double hValue();
double vValue();
void moveP(double direction, double speed, double dist);
void turnP(int dir, double speed, double dist);
