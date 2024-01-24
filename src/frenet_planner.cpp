#include "../include/frenet_planner.hpp"
#include <vector>
namespace plt = matplotlibcpp;
namespace frenet_planner {

FPList::FPList(double current_speed_in, double current_accel_in, double current_lat_pos_in, double current_lat_speed_in, 
            double current_lat_accel_in, double current_pos_in)
{
    current_speed = current_speed_in;
    current_lat_pos = current_lat_pos_in;
    current_lat_speed = current_lat_speed_in;
    current_lat_accel = current_lat_accel_in;
    current_pos = current_pos_in;
    current_accel = current_accel_in;
    for (double w = -MAX_ROAD_WIDTH; w < MAX_ROAD_WIDTH; w += SAMPLE_ROAD_WIDTH)
    {
        for (double ti = MIN_T; ti < MAX_T; ti += DT)
        {
            FrenetPath lat_samples = calculate_lat(w, ti);

            for (double tv = TARGET_SPEED - SAMPLE_TARGET_SPEED * N_S_SAMPLE; 
                tv < TARGET_SPEED + SAMPLE_TARGET_SPEED * N_S_SAMPLE; tv += SAMPLE_TARGET_SPEED)
            {
                FrenetPath path_samples = calculate_long(tv, ti, lat_samples);
                
                fplist_paths.push_back(path_samples);
            }
        }
    }

}

FrenetPath FPList::calculate_long(double speed, double time, FrenetPath lat_samples)
{
    FrenetPath tfp = lat_samples;
    polynomials::QuarticPolynomial long_qp(current_pos, current_speed, current_accel, speed, 0, time);
    int n = 1 + time / DT;
    tfp.t.resize(n);
	tfp.s.resize(n);
	tfp.s_d.resize(n);
	tfp.s_dd.resize(n);
	tfp.s_ddd.resize(n);
    
	for (int te = 0; te < n; te++)
	{
        tfp.t[te] = te * DT;
		tfp.s[te] = long_qp.calc_point(te * DT);
		tfp.s_d[te] = long_qp.calc_first_derivative(te * DT);
		tfp.s_dd[te] = long_qp.calc_second_derivative(te * DT);
		tfp.s_ddd[te] = long_qp.calc_third_derivative(te * DT);
	}
    
    tfp.Ti = (time);
	vecDouble s_ddd_vec = tfp.s_ddd;
	tfp.Js = (std::inner_product(s_ddd_vec.begin(), s_ddd_vec.end(), s_ddd_vec.begin(), 0));
	tfp.dss = std::pow((TARGET_SPEED - tfp.s_d.back()), 2);
	tfp.cv = (KJ * tfp.Js + KT * tfp.Ti + KD * tfp.dss);
    tfp.cf = (KLAT * tfp.cd + KLON * tfp.cv);
    return tfp;
}

void FrenetPath::plot_path()
{
	plt::plot(x, y);
}

FrenetPath FPList::calculate_lat(double dist, double time)
{
    FrenetPath fp;
    int n = 1 + time / DT;
    fp.t.resize(n);
    fp.d.resize(n);
    fp.d_d.resize(n);
    fp.d_dd.resize(n);
    fp.d_ddd.resize(n);

    polynomials::QuinticPolynomial lat_qp(current_lat_pos, current_lat_speed, current_lat_accel, dist, 0, 0, time);

    for (int s = 0; s < n; s++)
    {
        fp.t[s] = s * DT;
        fp.d[s] = lat_qp.calculate_point(s * DT);
        fp.d_d[s] = lat_qp.calculate_first_derivative(s * DT);
        fp.d_dd[s] = lat_qp.calculate_second_derivative(s * DT);
        fp.d_ddd[s] = lat_qp.calculate_third_derivative(s * DT);
    }

    vecDouble d_ddd_vec = fp.d_ddd;

    fp.Jp = (std::inner_product(d_ddd_vec.begin(), d_ddd_vec.end(), d_ddd_vec.begin(), 0));
    fp.Ti = time;
    fp.cd = (KJ * fp.Jp + KT * time + KD * std::pow((fp.d).back(), 2));
    return fp;
}

FrenetPath calculate_global_path(FrenetPath fp, cubic_spline::CubicSpline2D spline)
{
    fp.adding_global_path(spline);
    return fp;
}

void FrenetPath::adding_global_path(cubic_spline::CubicSpline2D spline)
{
    int n = s.size();
    x.resize(0);
	y.resize(0);

    for (int i = 0; i < n; i++)
    {
        double ix, iy;
        spline.calculatePositions(ix, iy, s[i]);
        if (ix == NONE)
        {
           break;;
        }
        double iyaw = spline.calc_yaw(s[i]);
        double fx = ix - d[i] * sin(iyaw);
		double fy = iy + d[i] * cos(iyaw);
        x.push_back(fx);
		y.push_back(fy);
    }

    yaw.resize(0);
	ds.resize(0);
    for (int i = 0; i < x.size() - 1; i++)
	{
        double dx = x[i + 1] - x[i];
		double dy = y[i + 1] - y[i];
        if (abs(dx) > 0.0001)
		{
			yaw.push_back(atan2(dy, dx));
		}
		else
		{
			yaw.push_back(0);
		}

        ds.push_back(sqrt(dx * dx + dy * dy));
    }

    if (s.size() == x.size())
	{
		return;
	}

    c.resize(0);

    for (int i = 0; i < (x.size() - 1) - 1; i++)
	{
        c.push_back((yaw[i + 1] - yaw[i]) / ds[i]);
	}
}

bool check_collision(double x, double y, std::vector<vecDouble> obstacles)
{
    for (int i = 0; i < obstacles.size(); i++)
    {
        double dist = sqrt(std::pow(x - obstacles[i][0], 2) + std::pow(y - obstacles[i][1], 2));
        if (dist <= std::pow(ROBOT_RADIUS, 2))
        {
            return 1;
        }
    }
    return 0;
} 

bool check_path(FrenetPath fp, std::vector<vecDouble> obstacles)
{
    for (int i = 0; i < fp.s_d.size(); i++)
    {
        if (fp.s_d[i] > MAX_SPEED)
        {
            return 0;
        }
    }
    
    for (int i = 0; i < fp.s_dd.size(); i++)
    {
        if (fp.s_dd[i] > MAX_ACCEL)
        {
            return 0;
        }
    }

    for (int i = 0; i < fp.c.size(); i++)
    {
        if (fp.c[i] > MAX_CURVATURE)
        {
            return 0;
        }
    }

    for (int i = 0; i < fp.x.size(); i++)
    {
        if(check_collision(fp.x[i], fp.y[i], obstacles))
        {
            return 0;
        }
    }

	return 1;
}

inline bool sortByCost(FrenetPath &a, FrenetPath &b)
{
	if (a.get_cf() != b.get_cf())
	{
		return a.get_cf() < b.get_cf();
	}
	else
	{
		double jerkCost1, jerkCost2;
		jerkCost1 = KLAT * a.get_Jp() + KLON * a.get_Js();
		jerkCost2 = KLAT * b.get_Jp() + KLON * b.get_Js();
		return jerkCost1 < jerkCost2;
	}
}

FrenetPath frenet_optimal_path(cubic_spline::CubicSpline2D spline, double current_pos, double current_speed, 
                                double current_accel, double current_lat_pos, double current_lat_speed,
                                double current_lat_accel, std::vector<vecDouble> obstacles)
{
    static FrenetPath bestpath;
    std::vector<FrenetPath> fplist = FPList(current_speed, current_accel, current_lat_pos, current_lat_speed, current_lat_accel, current_pos).fplist_paths;
    std::sort(fplist.begin(), fplist.end(), sortByCost);
    for (int i = 0; i < fplist.size(); i++)
    {
        fplist[i] = calculate_global_path(fplist[i], spline);
        
    }
    for (int i = 0; i < fplist.size(); i++)
    {
        if (check_path(fplist[i], obstacles))
        {
            bestpath = fplist[i];
            break;
        }
    }
    plt::cla();
    for (auto &fp : fplist)
	{
        fp.plot_path();        
	}
		
    return bestpath;
}

} // namespace frenet_planner

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
    N_S_SAMPLE = 1;
    ROBOT_RADIUS = 2.0;

    KJ = 0.1;
    KT = 0.1;
    KD = 1.0;
    KD_V = 0.5;
    KLAT = 1.0;
    KLON = 1.0;

    double current_pos = 0.0;
    double current_lat_pos = 0.0;
    double current_lat_speed = 0.0;
    double current_lat_accel = 0.0;
    double current_speed = 10 / 3.6;
    double current_accel = 0.0;

    vecDouble way_x{0, 10, 20, 30, 40, 45, 40, 30, 20, 
                    10, 0, -5, 0, 10, 20, 30, 40, 45, 30, 10, -5, -3};
    vecDouble way_y{0, -6, -10, -5, 0, 10, 20, 25, 
                    30, 35, 40, 50, 60, 65, 70, 65, 55, 45, 35, 20, 10, 3};
    std::vector<vecDouble> obstacles{{20.0, -10.0},
                                    {30.0, -5.0},
                                    {40.0, 40.0}};
    vecDouble rx, ry, ryaw, rk;
    double ds = 0.1;

    cubic_spline::CubicSpline2D spline = cubic_spline::calc_spline_course(way_x, way_y, rx, ry, ryaw, rk, ds);

    for (int step = 0; step < SIM_LOOP; step++) {
        frenet_planner::FrenetPath path = frenet_planner::frenet_optimal_path(spline, current_pos, current_speed, current_accel,
                                            current_lat_pos, current_lat_speed, current_lat_accel, obstacles);
        current_pos = path.s[1];
        current_lat_pos = path.d[1];
        current_lat_speed = path.d_d[1];
        current_lat_accel = path.d_dd[1];
        current_speed = path.s_d[1];
        current_accel = path.s_dd[1];
        plt::plot(rx,ry);
        plt::plot(path.x, path.y, "r--");
        vecDouble x1{path.x[1]};
        vecDouble y1{path.y[1]};
        vecDouble x1_y{cos(path.yaw[1])};
        vecDouble y1_y{sin(path.yaw[1])};
        plt::plot(x1, y1, "ro");
        vecDouble ob_x;
        vecDouble ob_y;
        for (int i = 0; i < obstacles.size(); i++)
        {
            ob_x.push_back(obstacles[i][0]);
            ob_y.push_back(obstacles[i][1]);
        }
        plt::plot(ob_x, ob_y, "xb");
        plt::quiver(x1, y1, x1_y, y1_y);
        plt::xlim(path.x[1] - 20, path.x[1] + 20);
        plt::ylim(path.y[1] - 20, path.y[1] + 20);
        plt::pause(0.001);
        double dist = std::pow(path.x[1] - rx.back(), 2) + std::pow(path.y[1] - ry.back(), 2);
        if (dist <= 1)
        {
            std::cout << "REACHED GOAL !!" << std::endl;
            break;
        }
    }
    return 0;
}