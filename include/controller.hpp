#include <tuple>
#include <vector>

double estimate_time(double x1,double  y1,double  x2,double  y2);
std::tuple<std::array<double, 16>, std::array<double, 16>, std::array<double, 4>, std::array<double, 4>> constructMatrix(double x1, double y1, double dx1, double dy1, double time1, double x2, double y2, double dx2, double dy2, double time2);
std::array<double, 16> InvertMatrix(const std::array<double, 16>& m);
std::array<double, 4> CalculateLine(std::array<double, 4>& m, std::array<double, 16>& bigMatrix);
std::tuple<double, double> updateSlope(double dx, double dy);
double findAngle(double dy, double dx);
namespace path {
    class PathController {
        public:
            std::vector<double> points = {0,0,0,8.6393798, 0};
            std::vector<double> lineX;
            std::vector<double> lineY;
            double timeInc = 0.5;
            double time = timeInc;
            void reset_and_add_point(double x1, double y1, double dx1, double dy1, double x2, double y2, double dx2, double dy2);
            void add_point(double x, double y, double dx, double dy);
            void makeLine();
            void changeIncrement(double value);
            std::tuple<double,double,double> nextCarrotPoint();
    };
}
std::tuple<double,double> findSlope(double angle);
void runPath(path::PathController AutonomousPath, double time);
void path1();

std::tuple<double,double,double,double,double,double> PidFollow(double currX,double currY,double currAngle,double carrotAdjustmentX,double carrotAdjustmentY,double carrotAdjustmentAngle,double errorDistance,double errorAngle,double integralDistance,double integralAngle);


namespace custom {
    class Odometry {
        public:
            double abs_x, abs_y;
            double curr_h, curr_v, curr_or;
            double prev_h, prev_v, prev_or;
            double delta_h, delta_v, delta_or;
            Odometry(double absolute_x, double absolute_y, double start_angle);
            std::tuple<double,double> get_abs();
            void set_pos(double absolute_x, double absolute_y);
            void update_currents_deltas();
            void calc_add_offsets();
            void update_previous();
            void print_vals();
    };
}


void testOdom();