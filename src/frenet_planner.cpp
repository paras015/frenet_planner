#include "../include/frenet_planner.hpp"
#include <vector>

namespace frenet_planner {


}

namespace plt = matplotlibcpp;

int main(int argc, char **argv)
{
    SIM_LOOP = 500;
    MAX_SPEED = 50.0 / 3.6;
    MAX_ACCEL = 2.0;
    MAX_CURVATURE = 1.0;
    MAX_ROAD_WIDTH = 7.0;
    SAMPLE_ROAD_WIDTH = 1.0;
    DT = 0.2;
    MAX_T = 5.0;
    MIN_T = 4.0;
    TARGET_SPEED = 30.0 / 3.6;
    SAMPLE_TARGET_SPEED = 5.0 / 3.6;
    ROBOT_RADIUS = 2.0;

    double current_pos = 0.0;
    double current_lat_pos = 0.0;
    double current_lat_speed = 0.0;
    double current_lat_accel = 0.0;
    double current_speed = 10 / 3.6;
    double current_accel = 0.0;

    vecDouble way_x{0.0, 10.0, 20, 30.0, 40, 45, 40, 30, 20, 10, 0, 10, 20};
    vecDouble way_y{0.0, -6.0, -10, -5, 0, 10, 20, 25, 30, 35, 40, 50, 60};
    vecDouble rx, ry, ryaw, rk;
    double ds = 0.1;

    cubic_spline::CubicSpline2D spline = cubic_spline::calc_spline_course(way_x, way_y, rx, ry, ryaw, rk, ds);

    // //Plotting Mechanism
    plt::plot(rx,ry);
    plt::show();

    // for (int step = 0; step < SIM_LOOP; step++) {
    //    frenet_planner::FrenetPath path = frenet_planner::frenet_optimal_path(spline, current_pos, current_speed, current_accel,
    //                                        current_lat_pos, current_lat_speed, current_lat_accel);

    //}
    return 0;
}
