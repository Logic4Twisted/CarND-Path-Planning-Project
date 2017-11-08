#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <math.h>
#include <iostream>
#include <vector>
#include <unordered_map>
#include "helper_functions.h"
#include "map.h"

const static double COLLISION = 1.0E6;
const static double PROJECT_GOAL = 1.0E6;
const static double TRAFIC_LAWS = 1.0E5;
const static double REACH_GOAL = 1.0E5;
const static double COMFORT = 1.0E4;
const static double EFFICIENCY = 1.0E2;
const static double LANE_CHANGE = 1.0E1;

const static double SPEED_LIMIT = 50.0; // mi/h
const static double SPEED_LIMIT_BUFFER = 3.0; // mi/h

const static double DELTA_T = 0.1; // seconds

const static double MAX_JERK = 50.0;
const static double MAX_ACCELERATION = 10.0;

const static double MIN_DISTANCE = 7.0; // meters
const static double RECOMMENDED_DISTANCE = 15.0; // meters

using namespace std;

class Trajectory
{
  public:
  deque<double> s;
  deque<double> d;
  deque<double> x;
  deque<double> y;
  deque<double> vs;
  deque<double> va;

  double start_s;
  double start_d;
  double final_s;
  double final_d;
  double final_v;

  double max_jerk_s;
  double max_acceleration_s;
  double max_velocity_s;

  double max_jerk_d;
  double max_acceleration_d;
  double max_velocity_d;

  double avg_speed;
  double max_speed;
  
  int lane_change;
  int lane;

  double cost(std::vector<std::vector<double>> sensor_fusion) const;
  double comfort_cost() const;
  double collision_cost(vector<vector<double>> sensor_fusion) const;
  double inefficiency_cost() const;
  double overspeeding_cost() const;
  static Trajectory create_CL_trajectory(CarLocation current, Target target, HighwayMap map, double T);
  static Trajectory create_trajectory(CarLocation current, Target target, HighwayMap map);
  static unordered_map<string, Trajectory> generate_trajectories(Trajectory previous, CarLocation current, HighwayMap map, int N_to_generate, double T);
  void printToStdout() const;
  vector<double> front();
  void pop_front();
  void push_back(vector<double> dp);
  void calculateStats();

private:
  double acceleration_cost() const;
  double jerk_cost() const;
};

class ImpossibleTrajectory: public Trajectory {
  double cost(std::vector<std::vector<double>> sensor_fusion) const;
  void printToStdout() const;
};

#endif /* TRAJECTORY_H_ */