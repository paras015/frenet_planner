#ifndef FRENET_PLANNER_HPP_
#define FRENET_PLANNER_HPP_

#include "../include/cublic_spline.hpp"
#include "matplotlibcpp.h"

int SIM_LOOP;
double MAX_SPEED;
double MAX_ACCEL;
double MAX_CURVATURE;
double MAX_ROAD_WIDTH;
double SAMPLE_ROAD_WIDTH;
double DT;
double MAX_T;
double MIN_T;
double TARGET_SPEED;
double SAMPLE_TARGET_SPEED;
double ROBOT_RADIUS;


namespace frenet_planner 
{

class FrenetPath 
{

};

FrenetPath frenet_optimal_path(cubic_spline::CubicSpline2D spline, double current_pos, double current_speed, 
                                double current_accel, double current_lat_pos, double current_lat_speed,
                                double current_lat_accel);
} // namespace frenet_planner

#endif //FRENET_PLANNER_HPP_
