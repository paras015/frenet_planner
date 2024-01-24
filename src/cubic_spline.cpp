#include "../include/cublic_spline.hpp"

namespace cubic_spline
{

void CubicSpline1D::init(vecDouble x_in, vecDouble y_in)
{
    x = x_in;
    y = y_in;
    nx = x.size();
    vecDouble h;
    for (int i = 1; i < x.size(); i++)
    {
		h.push_back(x[i] - x[i - 1]);
    }

    a = y;
    alpha.resize(nx, 0);
    l.resize(nx, 0);
    u.resize(nx, 0);
    z.resize(nx, 0);
    c.resize(nx, 0);
    b.resize(nx, 0);
    d.resize(nx, 0);

    for (int i = 1; i < nx - 1; i++)
    {
        alpha[i] = (3 / h[i]) * (a[i + 1] - a[i]) - (3 / h[i - 1]) * (a[i] - a[i - 1]);
    }

    l[0] = 1;
    u[0] = 0;
    z[0] = 0;

    for (int i = 1; i <= nx - 1; i++)
    {
        l[i] = 2 * (x[i + 1] - x[i - 1]) - h[i - 1] * u[i - 1];
        u[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    l[nx] = 1;
    z[nx] = 0;
    c[nx] = 0;

    for (int i = nx - 1; i >= 0; i--)
    {
        c[i] = z[i] - u[i] * c[i + 1];
        b[i] = (a[i + 1] - a[i]) / h[i] - h[i] * (c[i + 1] + 2 * c[i]) / 3;
        d[i] = (c[i + 1] - c[i]) / (3 * h[i]); 
    }

}

int CubicSpline1D::search_index(double t)
{
    auto it = upper_bound(x.begin(), x.end(), t);

	if (it == x.end())
	{
        return x.size() - 2;
	}

    return it - x.begin() - 1;
}

double CubicSpline1D::calculate(double t)
{
    if (x.size() == 0)
	{
		return NONE;
	}

	else if (t < x[0])
	{
		return NONE;
	}

	else if (t > x[nx - 1])
	{
		return NONE;
	}

	int i = search_index(t);
	double dx = t - x[i];

	if (i > nx - 1)
	{
		return NONE;
	}

	double result = a[i] + b[i] * dx + c[i] * dx * dx + d[i] * dx * dx * dx;

	return result;
}

double CubicSpline1D::calcd(double t)
{
	if (t < x[0])
	{
		return NONE;
	}
	else if (t > x[nx - 1])
	{
		return NONE;
	}
	int i = search_index(t);
	double dx = t - x[i];
	double result = b[i] + 2 * c[i] * dx + 3 * d[i] * dx * dx;

	return result;
}

double CubicSpline1D::calcdd(double t)
{
	if (t < x[0])
		return NONE;
	else if (t > x[nx - 1])
		return NONE;

	int i = search_index(t);

	double dx = t - x[i];
	double result = 2 * c[i] + 6 * d[i] * dx;

	return result;
}

vecDouble CubicSpline2D::calcS(vecDouble x, vecDouble y)
{
    vecDouble dx, dy, ds, s;

    for (int i = 1; i < x.size(); i++)
    {
        dx.push_back(x[i] - x[i - 1]);
    }

    for (int i = 1; i < y.size(); i++)
    {
        dy.push_back(y[i] - y[i - 1]);
    }

    for (int i = 0; i < dx.size(); i++)
    {
        ds.push_back(sqrt(dx[i] * dx[i] + dy[i] * dy[i]));
    }

    s.push_back(0);

    for (int i = 0; i < ds.size(); i++)
    {
        s.push_back(s.back() + ds[i]);
    }

    return s;
}

double CubicSpline2D::getLastS()
{
    return s.back();
}

double CubicSpline2D::calc_yaw(double t)
{
	double dx = sx.calcd(t);
	double dy = sy.calcd(t);
	double yaw = atan2(dy, dx);

	return yaw;
}

void CubicSpline2D::calculatePositions(double &x, double &y, double t)
{
    x = sx.calculate(t);
	y = sy.calculate(t);
}

double CubicSpline2D::calc_curvature(double t)
{
	double dx = sx.calcd(t);
	double ddx = sx.calcdd(t);

	double dy = sy.calcd(t);
	double ddy = sy.calcdd(t);

	double k = (ddy * dx - ddx * dy) / (dx * dx + dy * dy);

	return k;
}

CubicSpline2D calc_spline_course(vecDouble way_x, vecDouble way_y, vecDouble &rx, 
                                vecDouble &ry, vecDouble &ryaw, vecDouble &rk, double ds)
{
    CubicSpline2D spline(way_x, way_y);
    vecDouble s;
    double sRange = spline.getLastS();
    double sInc = 0;

    while (1)
    {
        if (sInc >= sRange)
        {
            break;
        }

        s.push_back(sInc);
        sInc = sInc + ds;
    }

    rx.resize(s.size());
    ry.resize(s.size());
    ryaw.resize(s.size());
    rk.resize(s.size());

    for (int i = 0; i < s.size(); i++)
    {
        double ix, iy;
        spline.calculatePositions(ix, iy, s[i]);
        rx[i] = ix;
		ry[i] = iy;
        ryaw[i] = spline.calc_yaw(s[i]);
		rk[i] = spline.calc_curvature(s[i]);
    }
    return spline;
}

} // namespace cubic_spline

