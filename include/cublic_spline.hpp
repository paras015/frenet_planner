#ifndef CUBIC_SPLINE_HPP_
#define CUBIC_SPLINE_HPP_

#include <eigen3/Eigen/Dense>
#include <vector>

#define NONE -1e9

using vecDouble = std::vector<double>;

namespace cubic_spline {

class CubicSpline1D
{
private:
    vecDouble x, y;
    vecDouble a, b, c, d, w;
    vecDouble alpha, l, u, z;
    int nx;
    int search_index(double t);

public:
    void init(vecDouble x_in, vecDouble y_in);
    double calculate(double t);
};

class CubicSpline2D
{
private:
    vecDouble x, y, s;
    CubicSpline1D sx, sy;

public:
    CubicSpline2D(vecDouble way_x, vecDouble way_y)
    {
        x = way_x;
        y = way_y;
        s = calcS(x, y);
        sx.init(s, x);
        sy.init(s, y);
    }

    vecDouble calcS(vecDouble x, vecDouble y);
    double getLastS();
    void calculatePositions(double &x, double &y, double t);
};

CubicSpline2D calc_spline_course(vecDouble way_x, vecDouble way_y, vecDouble &rx, 
                                vecDouble &ry, vecDouble &ryaw, vecDouble &rk, double ds);
} //namespace cubic_spline

#endif //CUBIC_SPLINE_HPP_