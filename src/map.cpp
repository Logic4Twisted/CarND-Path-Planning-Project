#include "map.h"
#include <vector>
#include "spline.h"
#include "helper_functions.h"

using namespace std;

vector<double> HighwayMap::getXY(double s, double d) {
	int prev_wp = -1;
	int map_size = waypoints_x.size();

	while(s > waypoints_s[prev_wp+1] && (prev_wp < (int)(waypoints_s.size()-1) ))
	{
		prev_wp++;
	}
	vector<int> wps;
	wps.push_back((prev_wp - 1 + map_size)%map_size);
	wps.push_back(prev_wp%map_size);
	wps.push_back((prev_wp+1)%map_size);
	wps.push_back((prev_wp+2)%map_size);

	
	vector<double> ptsx, ptsy, ptsdx, ptsdy, ptss;
	for (int i = 0; i < wps.size(); i++) {
		int wp = wps[i];
		ptsx.push_back(waypoints_x[wp]);
		ptsy.push_back(waypoints_y[wp]);
		ptsdx.push_back(waypoints_dx[wp]);
		ptsdy.push_back(waypoints_dy[wp]);

		double wp_s = waypoints_s[wp];
		if (i == 0 && wp == map_size - 1) wp_s -= TRACK_LEN;
		if (i > 0 && ptss[i-1] > waypoints_s[wp]) wp_s += TRACK_LEN;
		ptss.push_back(wp_s);
	}
	tk::spline spline_x_s;
	spline_x_s.set_points(ptss, ptsx);

	tk::spline spline_y_s;
	spline_y_s.set_points(ptss, ptsy);

	tk::spline spline_dx_s;
	spline_dx_s.set_points(ptss, ptsdx);

	tk::spline spline_dy_s;
	spline_dy_s.set_points(ptss, ptsdy);

	double x = spline_x_s(s) + d*spline_dx_s(s);
	double y = spline_y_s(s) + d*spline_dy_s(s);

	return {x,y};
}

void HighwayMap::addWaypoint(double x, double y, double s, double dx, double dy) {
	waypoints_x.push_back(x);
	waypoints_y.push_back(y);
	waypoints_s.push_back(s);
	waypoints_dx.push_back(dx);
	waypoints_dy.push_back(dy);
}

vector<double> HighwayMap::getFrenet(double x, double y, double theta) {
	int next_wp = NextWaypoint(x,y, theta);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = waypoints_x.size()-1;
	}

	double n_x = waypoints_x[next_wp]-waypoints_x[prev_wp];
	double n_y = waypoints_y[next_wp]-waypoints_y[prev_wp];
	double x_x = x - waypoints_x[prev_wp];
	double x_y = y - waypoints_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-waypoints_x[prev_wp];
	double center_y = 2000-waypoints_y[prev_wp];
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
		frenet_s += distance(waypoints_x[i],waypoints_y[i],waypoints_x[i+1],waypoints_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}

int HighwayMap::NextWaypoint(double x, double y, double theta) {
	int closestWaypoint = ClosestWaypoint(x,y);

	double map_x = waypoints_x[closestWaypoint];
	double map_y = waypoints_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = fmod(fabs(theta-heading), 2*pi());

	if(angle > pi()/4 && angle < 7*pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;
}

int HighwayMap::ClosestWaypoint(double x, double y) {
	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < waypoints_x.size(); i++)
	{
		double map_x = waypoints_x[i];
		double map_y = waypoints_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;
}