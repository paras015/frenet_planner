#ifndef POLYNOMIALS_HPP_
#define POLYNOMIALS_HPP_

#include <eigen3/Eigen/Dense>

namespace polynomials
{

class QuinticPolynomial
{
private:
    double xs, vxs, axs, xe, vxe, axe, a0, a1, a2, a3, a4, a5;
public:
    QuinticPolynomial(double current_lat_pos, double current_lat_speed, double current_lat_accel, 
                        double dist, double dist_d, double dist_dd, double T);
    double calculate_point(double t);
    double calculate_first_derivative(double t);
    double calculate_second_derivative(double t);
    double calculate_third_derivative(double t);
};

class QuarticPolynomial
{
private:
    double xs, vxs, axs, vxe, axe, a0, a1, a2, a3, a4;
public:
    QuarticPolynomial(double xs_in, double vxs_in, double axs_in, double vxe_in, double axe_in, double T);
    double calc_point(double);
    double calc_first_derivative(double);
    double calc_second_derivative(double);
    double calc_third_derivative(double);
};

} // namespace polynomials

#endif // POLYNOMIALS_HPP_