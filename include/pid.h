#include "config.h"
#include "myFunctions.h"
#include <tuple>

std::tuple<double, double, double> PidCalc(double kp, double ki, double kd, double target, double time, double current, double prev_error, double integral);

int PidMoveTo(double targetPos, double kp, double ki, double kd);
int PidTurnTo(double targetHeading);
std::tuple<bool> PidTurnTo(double targetHeading, bool armIntake);
int PidTurnLeftTo(double targetHeading);
int PidTurnRightTo(double targetHeading);
std::tuple<double,double,double,double,double,double> PidFollow(double currX,double currY,double currAngle,double carrotAdjustmentX,double carrotAdjustmentY,double carrotAdjustmentAngle,double errorDistance,double errorAngle,double integralDistance,double integralAngle);
void AutoPidTuning(double targetParam,double iterations, double rangeLower, double rangeHigher);