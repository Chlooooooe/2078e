#include <cmath>
#include <tuple>
#include <vector>

#include "tracking.hpp"
#include "pid.h"
#include "myFunctions.h"
#include <tuple>



std::tuple<std::array<double, 16>, std::array<double, 16>, std::array<double, 4>, std::array<double, 4>> constructMatrix(double x1, double y1, double dx1, double dy1, double time1, double x2, double y2, double dx2, double dy2, double time2){
    
    std::array<double, 16> matrixX4x4 =  {1, time1, time1*time1, time1*time1*time1, 1,time2,time2*time2,time2*time2*time2,0,1,2*time1,3*time1*time1,0,1,2*time2,3*time2*time2};
    std::array<double, 16> matrixY4x4 =  {1, time1, time1*time1, time1*time1*time1, 1,time2,time2*time2,time2*time2*time2,0,1,2*time1,3*time1*time1,0,1,2*time2,3*time2*time2};
    
    std::array<double, 4> matrixX4x1 = {x1, x2, dx1, dx2};
    std::array<double, 4> matrixY4x1 = {y1, y2, dy1, dy2};
	
    
    return std::make_tuple(matrixX4x4,matrixY4x4,matrixX4x1,matrixY4x1);
}



std::array<double, 16> InvertMatrix(const std::array<double, 16>& m){       // Just converts a 4x4 matrix in a format of 01 02 03 04 11 12 13 14 21 22 23 24 31 32 33 34 format (aka just length 16 array) into it's inverse.
    std::array<double, 16> invOut;
	std::array<double, 16> inv; 
	double det;
    int i;

    inv[0] = m[5]  * m[10] * m[15] -                                        //formula for each element in the inverse
             m[5]  * m[11] * m[14] - 
             m[9]  * m[6]  * m[15] + 
             m[9]  * m[7]  * m[14] +
             m[13] * m[6]  * m[11] - 
             m[13] * m[7]  * m[10];

    inv[4] = -m[4]  * m[10] * m[15] + 
              m[4]  * m[11] * m[14] + 
              m[8]  * m[6]  * m[15] - 
              m[8]  * m[7]  * m[14] - 
              m[12] * m[6]  * m[11] + 
              m[12] * m[7]  * m[10];

    inv[8] = m[4]  * m[9] * m[15] - 
             m[4]  * m[11] * m[13] - 
             m[8]  * m[5] * m[15] + 
             m[8]  * m[7] * m[13] + 
             m[12] * m[5] * m[11] - 
             m[12] * m[7] * m[9];

    inv[12] = -m[4]  * m[9] * m[14] + 
               m[4]  * m[10] * m[13] +
               m[8]  * m[5] * m[14] - 
               m[8]  * m[6] * m[13] - 
               m[12] * m[5] * m[10] + 
               m[12] * m[6] * m[9];

    inv[1] = -m[1]  * m[10] * m[15] + 
              m[1]  * m[11] * m[14] + 
              m[9]  * m[2] * m[15] - 
              m[9]  * m[3] * m[14] - 
              m[13] * m[2] * m[11] + 
              m[13] * m[3] * m[10];

    inv[5] = m[0]  * m[10] * m[15] - 
             m[0]  * m[11] * m[14] - 
             m[8]  * m[2] * m[15] + 
             m[8]  * m[3] * m[14] + 
             m[12] * m[2] * m[11] - 
             m[12] * m[3] * m[10];

    inv[9] = -m[0]  * m[9] * m[15] + 
              m[0]  * m[11] * m[13] + 
              m[8]  * m[1] * m[15] - 
              m[8]  * m[3] * m[13] - 
              m[12] * m[1] * m[11] + 
              m[12] * m[3] * m[9];

    inv[13] = m[0]  * m[9] * m[14] - 
              m[0]  * m[10] * m[13] - 
              m[8]  * m[1] * m[14] + 
              m[8]  * m[2] * m[13] + 
              m[12] * m[1] * m[10] - 
              m[12] * m[2] * m[9];

    inv[2] = m[1]  * m[6] * m[15] - 
             m[1]  * m[7] * m[14] - 
             m[5]  * m[2] * m[15] + 
             m[5]  * m[3] * m[14] + 
             m[13] * m[2] * m[7] - 
             m[13] * m[3] * m[6];

    inv[6] = -m[0]  * m[6] * m[15] + 
              m[0]  * m[7] * m[14] + 
              m[4]  * m[2] * m[15] - 
              m[4]  * m[3] * m[14] - 
              m[12] * m[2] * m[7] + 
              m[12] * m[3] * m[6];

    inv[10] = m[0]  * m[5] * m[15] - 
              m[0]  * m[7] * m[13] - 
              m[4]  * m[1] * m[15] + 
              m[4]  * m[3] * m[13] + 
              m[12] * m[1] * m[7] - 
              m[12] * m[3] * m[5];

    inv[14] = -m[0]  * m[5] * m[14] + 
               m[0]  * m[6] * m[13] + 
               m[4]  * m[1] * m[14] - 
               m[4]  * m[2] * m[13] - 
               m[12] * m[1] * m[6] + 
               m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] + 
              m[1] * m[7] * m[10] + 
              m[5] * m[2] * m[11] - 
              m[5] * m[3] * m[10] - 
              m[9] * m[2] * m[7] + 
              m[9] * m[3] * m[6];

    inv[7] = m[0] * m[6] * m[11] - 
             m[0] * m[7] * m[10] - 
             m[4] * m[2] * m[11] + 
             m[4] * m[3] * m[10] + 
             m[8] * m[2] * m[7] - 
             m[8] * m[3] * m[6];

    inv[11] = -m[0] * m[5] * m[11] + 
               m[0] * m[7] * m[9] + 
               m[4] * m[1] * m[11] - 
               m[4] * m[3] * m[9] - 
               m[8] * m[1] * m[7] + 
               m[8] * m[3] * m[5];

    inv[15] = m[0] * m[5] * m[10] - 
              m[0] * m[6] * m[9] - 
              m[4] * m[1] * m[10] + 
              m[4] * m[2] * m[9] + 
              m[8] * m[1] * m[6] - 
              m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];   //determinant, that's just how you calculate it ;-;

    det = 1.0 / det;                                                        //search up inverse matrix formula and you'll understand why you need to do this

    for (i = 0; i < 16; i++){                                               //1/inv * matrix, anyways this loops through all 16 elements
        invOut[i] = inv[i] * det;
	}
	return invOut;
}



std::tuple<double, double> updateSlope(double dx, double dy) {                  //Update slope is like finding a unit vector, but the unit vector's length is 8.639 (1/10 speed of the robot), instead of 1
                                                                                //This allows us to put any dx, dy when inputting points without worrying about 
    double slope = 8.6393798;//Unit vector magnitude                            //Why this? Form my testing, this gives tight, but not too tight turns. So- using 1 makes the turn on a scale of ~2 inches, and this is ~1-2 feet, while 86 is more like a 20 feet turn (most of the time in circle) Go to Dm for documentation.
    
    slope /= std::pow((std::pow((dx),2) + std::pow((dy),2)),0.5);    //Finds the scalar. Math and formula representation in Discord documentation
    
    
    return std::make_tuple(dx*slope, dy*slope);                         //Scales and returns dx, dy
}



double findRobotHeading(double dy, double dx) { //Returns 
    double angle;

    if (dx == 0.0)                              //Just a lot of logic, it's in docs if you need to know how it's done.
    {
        if (dy > 0.0) 
        {angle = 90;}
        else 
        {angle = 270;}
    }
    else{angle = fabs((atan(dy/dx))*180/M_PI-90);}

    if (dx<0) {angle += 180;}


    return angle;

}



std::array<double, 4> CalculateLine(std::array<double, 4>& m, std::array<double, 16>& bigMatrix){ //It's another way to say "I am multiplying two matrixes together". A 4x1 with a 4x4
	std::array<double, 4> outputMatrix;
	double sum = 0.0;
	for (int i = 0; i < 4; i++){                //loops through the four rows of 4x4
		sum = 0.0;
		for (int j = 0; j < 4; j++){            //loops through the columns 4x1 and 4x4
			sum += bigMatrix[4*i + j] * m[j];
		}
		outputMatrix[i] = sum;
	}
	return outputMatrix;
}



double estimate_time(double x1,double  y1,double  x2,double  y2){           //Estimation of time required to drive to the point; given the two points.
	return std::pow((std::pow((x2-x1),2) + std::pow((y2-y1),2)),0.5)/7.77544182;
}

namespace path {
    class PathController {

        public:
            std::vector<double> points = {0,0,0,8.6393798, 0}; // Every 5 values is a point
            std::vector<double> lineX; // Every 4 values is a line
            std::vector<double> lineY; // Every 4 values is a line
            double timeInc = 0.5;
            double time = timeInc;
            
            void reset_and_add_point(double x1, double y1, double dx1, double dy1, double x2, double y2, double dx2, double dy2){
                std::tie(dx1,dy1) = updateSlope(dx1,dy1);
                points = {x1,y1,dx1,dy1,0};
                lineX = {};
                lineY = {};
                double t = estimate_time(x1,y1,x2,y2);
                points.push_back(x2);
                points.push_back(y2);
                std::tie(dx2,dy2) = updateSlope(dx2,dy2);
                points.push_back(dx2);
                points.push_back(dy2);
                points.push_back(t);
                makeLine();
            }
            
            void add_point(double x, double y, double dx, double dy){
                double time = points[points.size()-1] + estimate_time(points[points.size()-5],points[points.size()-4],x,y);
                points.push_back(x);
                points.push_back(y);
                std::tie(dx,dy) = updateSlope(dx,dy);
                points.push_back(dx);
                points.push_back(dy);
                points.push_back(time);
                makeLine();
            }

            void makeLine() {
                double x1 = points[points.size()-10];
                double y1= points[points.size()-9];
                double dx1= points[points.size()-8];
                double dy1= points[points.size()-7];
                double time1= points[points.size()-6];
                double x2= points[points.size()-5];
                double y2= points[points.size()-4];
                double dx2= points[points.size()-3];
                double dy2= points[points.size()-2];
                double time2= points[points.size()-1];
                std::array<double, 16> matrixX4x4 =  {1, time1, time1*time1, time1*time1*time1, 1,time2,time2*time2,time2*time2*time2,0,1,2*time1,3*time1*time1,0,1,2*time2,3*time2*time2};
                std::array<double, 16> matrixY4x4 =  {1, time1, time1*time1, time1*time1*time1, 1,time2,time2*time2,time2*time2*time2,0,1,2*time1,3*time1*time1,0,1,2*time2,3*time2*time2};
                std::array<double, 4> matrixX4x1 = {x1, x2, dx1, dx2};
                std::array<double, 4> matrixY4x1 = {y1, y2, dy1, dy2};
                std::array<double, 16> invmatrixX4x4 = InvertMatrix(matrixX4x4);
                std::array<double, 16> invmatrixY4x4 = InvertMatrix(matrixY4x4);
                std::array<double, 4> X = CalculateLine(matrixX4x1,invmatrixX4x4);
                std::array<double, 4> Y = CalculateLine(matrixY4x1,invmatrixY4x4);
                for(int i = 0; i < 4; i++){
                    lineX.push_back(X[i]);
                }
                for(int i = 0; i < 4; i++){
                    lineY.push_back(Y[i]);
                }
            }
            
            void changeIncrement(double value){
                timeInc = value;    
            }
            
            std::tuple<double,double,double> nextCarrotPoint(){
                time += timeInc;
                int i = 5;
                while(i < points.size()){
                    if(time>points[i-1]&&time<points[i+4]){
                        break;
                    }
                    i += 5;
                }
                if (time > points[points.size()-1]){
                    time = points[points.size()-1];
                    i = points.size()-5;
                }
                i *= 0.8;
                double x = lineX[i-4] + lineX[i-3]*time + lineX[i-2]*time*time + lineX[i-1]*time*time*time;
                double y = lineY[i-4] + lineY[i-3]*time + lineY[i-2]*time*time + lineY[i-1]*time*time*time;
                return std::make_tuple(x,y,findRobotHeading((lineY[i-3] + 2*lineY[i-2]*time + 3*lineY[i-1]*time*time),(lineX[i-3] + 2*lineX[i-2]*time + 3*lineX[i-1]*time*time)));
            }


    };
}

std::tuple<double,double> findSlope(double angle){
    double dx, dy;
    if (angle<90){
        angle = fabs(angle -90);
    }else{
        angle = fabs(angle -450);
    }
    if(angle == 90){
        dx = 0;
        dy = 8.6393798;
    }
    else if (angle == 270){
        dx = 0;
        dy = -8.6393798;
    }
    else {
        angle = angle*M_PI/180;
        dx = 8.6393798/std::pow((1+std::pow(tan(angle),2)),0.5);
        dy = 8.6393798 * tan(angle) /std::pow((1+std::pow(tan(angle),2)),0.5);
    }
    if (angle>1.57079633 && angle < 4.71238899){
        dx = -dx;
        dy = -dy;
    }

    return std::make_tuple(dx,dy);
}


std::tuple<double,double,double,double,double,double> PidFollow(double currX,double currY,double currAngle,double carrotAdjustmentX,double carrotAdjustmentY,double carrotAdjustmentAngle,double errorDistance,double errorAngle,double integralDistance,double integralAngle){
	double outputAngle,outputMove;
	std::tie(outputAngle,errorAngle,integralAngle) = PidCalc(1.2, 0.0, 0.0, carrotAdjustmentAngle, 5,currAngle , errorAngle, integralAngle);
	double moveTo = std::pow((std::pow((currX-carrotAdjustmentX),2) + std::pow((currY-carrotAdjustmentY),2)),0.5);
	std::tie(outputMove,errorDistance,integralDistance) = PidCalc(4, 0.0, 0.0, moveTo, 5, 0, errorDistance, integralDistance);
	return std::make_tuple(outputAngle,outputMove,errorAngle,errorDistance,integralAngle,integralDistance);
}
namespace custom {
    class Odometry {
    
        public:
            
            double abs_x, abs_y;

            Odometry(double absolute_x, double absolute_y, double start_angle) {
                hwheel.set_position(0);
                vwheel.set_position(0);
                abs_x = absolute_x;
                abs_y = absolute_y;
                prev_h = hValue();
                prev_v = vValue();
                prev_or = start_angle * M_PI / 180;
            }

            std::tuple<double,double> get_abs(){
                return std::make_tuple(abs_x,abs_y);
            }
            
            void set_pos(double absolute_x, double absolute_y) {
                abs_x = absolute_x;
                abs_y = absolute_y;
            }

            void update_currents_deltas() {
                curr_h = hValue();
                curr_v = vValue();
                curr_or = getAngle();
                delta_h = curr_h - prev_h;
                delta_v = curr_v - prev_v;
                delta_or = curr_or - prev_or;
            }
            
            void calc_add_offsets() {
                
                // calculate local offset since previous
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

            void update_previous() {
                prev_h = curr_h;
                prev_v = curr_v;
                prev_or = curr_or;
            }

            void print_vals() {
                printf("\nX: %f     ", abs_x);
                printf("Y: %f ", abs_y);
            }
        
        private:
            double curr_h, curr_v, curr_or;
            double prev_h, prev_v, prev_or;
            double delta_h, delta_v, delta_or;
    };
}


void testOdom() {
    
    double abs_x = 0;
    double abs_y = 0;
    double prev_or = 0;
    double prev_h = 0;
    double prev_v = 0;
    
    hwheel.set_position(0);
    vwheel.set_position(0);

    while(true) {
        
        double curr_h = hValue();
        double curr_v = vValue();
        double curr_or = getAngle();

        // calculate change in orientation
        double delta_or = curr_or - prev_or;

        // calculate wheel movement since previous
        double delta_h = curr_h - prev_h;
        double delta_v = curr_v - prev_v;

        // calculate local offset since previous
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
        // abs_x += offset_global_x;
        // abs_y += offset_global_y;

        // TESTING
		pros::lcd::print(0, "x: %f", abs_x);
		pros::lcd::print(1, "y: %f", abs_y);
		pros::lcd::print(2, "xRot: %f", hwheel.get_position()/36000.0);
		pros::lcd::print(3, "yRot: %f", vwheel.get_position()/36000.0);

        // update previous values
        prev_h = curr_h;
        prev_v = curr_v;
        prev_or = curr_or;

        delay(5);

    }
    
}

void runPath(path::PathController AutonomousPath, double time){
    path::PathController Adjustment;
    Adjustment.changeIncrement(0.1);
    custom::Odometry odom(0,0,0);
    double carrotPathX, carrotPathY, carrotPathDx, carrotPathDy, carrotPathAngle;
    double carrotAdjustmentX, carrotAdjustmentY, carrotAdjustmentAngle;
    double currX, currY, currDx, currDy;
    double errorDistance, errorAngle, integralDistance,integralAngle;
    double outputAngle,outputMove;
    for(int i = 0; i < time*1000;i+=50){
        std::tie(carrotPathX,carrotPathY,carrotPathAngle) = AutonomousPath.nextCarrotPoint();
        integralDistance = 0;                               //reset integral here because integral shouldn't grow too large 
        integralAngle = 0;                                  //carrot point change feels like a good place to zero integral
        for(int j = 0; j < 2; j++){
            std::tie(currDx,currDy) = findSlope(inertial.get_heading());
            std::tie(carrotPathDx,carrotPathDy) = findSlope(carrotPathAngle);
            Adjustment.reset_and_add_point(currX,currY,currDx,currDy,carrotPathX,carrotPathY,carrotPathDx,carrotPathDy);
            for(int k = 0; k < 25; k+=5){
                odom.update_currents_deltas();
                odom.calc_add_offsets();
                odom.update_previous();
                std::tie(currX,currY) = odom.get_abs();
                std::tie(carrotAdjustmentX, carrotAdjustmentY,carrotAdjustmentAngle) = Adjustment.nextCarrotPoint();
                std::tie(outputAngle,outputMove,errorAngle,errorDistance,integralAngle,integralDistance) = PidFollow(currX,currY,inertial.get_heading(),carrotAdjustmentX,carrotAdjustmentY,carrotAdjustmentAngle,errorDistance,errorAngle,integralDistance,integralAngle);
                if (outputAngle<0){
                    left.move(outputMove+outputAngle);
                }else{
                    right.move(outputMove-outputAngle);
                }
                pros::delay(10);
            }
        }
    }
}
void path1(){
	path::PathController AutonomousPath;
	AutonomousPath.add_point(-24,24,-1,1);
    AutonomousPath.add_point(-10,48,0.5,2);
    AutonomousPath.add_point(20,70,3,0.5);
    AutonomousPath.add_point(50,60,0,-2);
    AutonomousPath.add_point(20,30,-1,-2);
    
    

	runPath(AutonomousPath,15);
}


