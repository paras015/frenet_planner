#ifndef FRENET_PLANNER_HPP_
#define FRENET_PLANNER_HPP_

#include "../include/cublic_spline.hpp"
#include "matplotlibcpp.h"
#include "../include/polynomials.hpp"

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
double N_S_SAMPLE;
double ROBOT_RADIUS;
double KJ;
double KT;
double KD;
double KD_V;
double KLAT;
double KLON;


namespace frenet_planner 
{

class FrenetPath 
{
public:
    vecDouble t, d, d_d, d_dd, d_ddd, s, s_d, s_dd, s_ddd, x, y, yaw, ds, c;

    double Js, Jp, cd, cv, cf, Ti, dss;

    void plot_path();
    void adding_global_path(cubic_spline::CubicSpline2D spline);
    double get_cf();
    double get_Jp();
	double get_Js();
};

class FPList
{
public:
    FPList(double current_speed_in, double current_accel_in, double current_lat_pos_in, double current_lat_speed_in, 
            double current_lat_accel_in, double current_pos_in);
    FrenetPath calculate_lat(double dist, double time);
    FrenetPath calculate_long(double speed, double time, FrenetPath lat_samples);
    
    double current_speed;
    double current_lat_pos;
    double current_lat_speed;
    double current_lat_accel;
    double current_pos;
    double current_accel;
    std::vector<FrenetPath> fplist_paths;
    std::vector<FrenetPath> fplist_lat;
    std::vector<FrenetPath> fplist_long;
    
};

inline double FrenetPath::get_cf()
{
	return cf;
}
inline double FrenetPath::get_Jp()
{
	return Jp;
}
inline double FrenetPath::get_Js()
{
	return Js;
}

bool check_path(FrenetPath fp, std::vector<vecDouble> obstacles);
bool check_collision(double x, double y, std::vector<vecDouble> obstacles);
FrenetPath calculate_global_path(FrenetPath fp, cubic_spline::CubicSpline2D spline);
FrenetPath frenet_optimal_path(cubic_spline::CubicSpline2D spline, double current_pos, double current_speed, 
                                double current_accel, double current_lat_pos, double current_lat_speed,
                                double current_lat_accel, std::vector<vecDouble> obstacles);
} // namespace frenet_planner

#endif //FRENET_PLANNER_HPP_