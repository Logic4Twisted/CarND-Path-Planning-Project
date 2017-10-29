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
const static double LANE_CHANGE = 1.0E1;

const static double SPEED_LIMIT = 50.0; // mi/h

const static double SPEED_LIMIT_BUFFER = 1.0; // mi/h

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
  double overspeeding_cost() const;
  static Trajectory create_CL_trajectory(CarLocation current, Target target, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
  static Trajectory create_trajectory(CarLocation current, Target target, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
};

#endif /* TRAJECTORY_H_ */