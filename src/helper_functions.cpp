#include "helper_functions.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <math.h>

using namespace std;
using namespace Eigen;

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double miph_to_kmph(double miph) {
	return miph*1.60934;
}

double miph_to_mps(double miph) {
	return miph_to_kmph(miph)/3.6;
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = fabs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

vector<double> closest_in_front(vector<vector<double>> sensor_fusion, int lane, double s, double range) {
  vector<double> result;
  double curr_min = range;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    double d = sensor_fusion[i][6];
    if (d > 4.0*lane && d < 4.0*lane+4.0) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double v = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];
      double dist = fmod(check_car_s - s + TRACK_LEN, TRACK_LEN);
      if (dist < curr_min) {
        curr_min = dist;
        result.clear();
        result.push_back(i);
        result.push_back(check_car_s);
        result.push_back(v);
      }
    }
  }
  return result;
}

// Calculates Jerk Minimizing Trajectory for start, end and T.
vector<double> jmt(vector<double> start, vector<double> final, double T) {
  assert(T > 0.0);
  assert(start.size() == 3);
  assert(final.size() == 3);

  double a0 = start[0];
  double a1 = start[1];
  double a2 = start[2]/2.0;

  double c0 = a0 + a1*T + a2*pow(T, 2.0);
  double c1 = a1 + 2.0*a2*T;
  double c2 = 2.0*a2;

  MatrixXd B = MatrixXd(3,1);
  B << final[0] - c0, final[1] - c1, final[2] - c2;

  MatrixXd A = MatrixXd(3,3);
  A << pow(T, 3.0), pow(T, 4.0), pow(T, 5.0),
       3.0*pow(T, 2.0), 4.0*pow(T, 3.0), 5.0*pow(T, 4.0),
       6.0*T, 12.0*pow(T, 2.0), 20.0*pow(T, 3.0);

  MatrixXd X = A.inverse() * B;

  vector<double> result;
  result.push_back(a0);
  result.push_back(a1);
  result.push_back(a2);
  result.push_back(X(0,0));
  result.push_back(X(1,0));
  result.push_back(X(2,0));
  return result;
}

double eval(vector<double> coeff, double x) {
  assert(coeff.size() > 0);
  double result = coeff[0];
  for (int i = 1; i < coeff.size(); i++) {
    result += coeff[i]*pow(x, i);
  }
  return result;
}

vector<double> dif(vector<double> coeff) {
  assert(coeff.size() > 0);
  vector<double> result;
  for (int i = 1; i < coeff.size(); i++) {
    result.push_back(coeff[i]*i);
  }
  return result;
}