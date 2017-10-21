#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <math.h>
#include <iostream>
#include <vector>
#include "car_location.h"

const static double COLLISION = 1.0E6;
const static double DANGER = 1.0E5;
const static double REACH_GOAL = 1.0E5;
const static double COMFORT = 1.0E4;
const static double EFFICIENCY = 1.0E3;
const static double LANE_CHANGE = 1.0E2;

using namespace std;

class Trajectory
{
  public:
  std::vector<double> s;
  std::vector<double> d;
  std::vector<double> x;
  std::vector<double> y;
  double start_s;
  double start_d;
  double final_s;
  double final_d;
  double final_v;
  int lane_change;
  int lane;

  double cost(std::vector<std::vector<double>> sensor_fusion) const;
  double comfort_cost() const;
  std::vector<double> closest_in_front(std::vector<std::vector<double>> sensor_fusion, double end_path_d, double end_path_s) const;
  double collision_cost(vector<vector<double>> sensor_fusion) const;
  double inefficiency_cost() const;

};

#endif /* TRAJECTORY_H_ */