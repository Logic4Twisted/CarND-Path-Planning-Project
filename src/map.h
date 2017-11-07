#ifndef MAP_H_
#define MAP_H_

#include <iostream>
#include <vector>
using namespace std;

class HighwayMap {
	vector<double> waypoints_x;
	vector<double> waypoints_y;
	vector<double> waypoints_dx;
	vector<double> waypoints_dy;
	vector<double> waypoints_s;

	int NextWaypoint(double x, double y, double theta);
	int ClosestWaypoint(double x, double y);

	public:
	vector<double> getXY(double s, double d);
	void addWaypoint(double x, double y, double s, double dx, double dy);
	vector<double> getFrenet(double x, double y, double theta);
};

#endif /* MAP_H_ */