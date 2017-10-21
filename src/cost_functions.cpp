#include "cost_functions.h"
#include <vector>

using namespace std;

double Trajectory::cost (vector<vector<double>> sensor_fusion) const {
  double result = 0.0;
  result += collision_cost(sensor_fusion);
  result += inefficiency_cost();
  result += comfort_cost();
  result += overspeeding_cost();
  return result;
}

double Trajectory::comfort_cost() const{
  return abs(lane_change)*LANE_CHANGE;
}

vector<double> Trajectory::closest_in_front(vector<vector<double>> sensor_fusion, double end_path_d, double end_path_s) const {
  vector<double> result;

  for (int i = 0; i < sensor_fusion.size(); i++) {
    double d = sensor_fusion[i][6];
    if (d < (4*lane + 2) && d > (4*lane - 2)) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double check_speed = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];

      check_car_s += ((double)50*0.02*check_speed);

      if ((check_car_s > end_path_s) && (result.size() == 0 || result[1] > check_car_s)) {
        result.clear();
        result.push_back(i);
        result.push_back(check_car_s);
        result.push_back(check_speed);
      }
    }
  }
  return result;
}

double Trajectory::collision_cost(vector<vector<double>> sensor_fusion) const {
  double result = 1.0;
  if (lane < 0 or lane >= 3) {
    result += 1;
  }
  double min_distance = 1000.0;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    double id = sensor_fusion[i][0];
    double x_ = sensor_fusion[i][1];
    double y_ = sensor_fusion[i][2];
    double d_ = sensor_fusion[i][6];
    double s_ = sensor_fusion[i][5];
    double vx = sensor_fusion[i][3];
    double vy = sensor_fusion[i][4];
    double v = sqrt(vx*vx + vy*vy);
    /*
    for (int j = 0; j < x.size(); j++) {
      double current_x = x_ + vx*j*0.02;
      double current_y = y_ + vy*j*0.02;
      vector<double> sd = getFrenet(current_x, )
    }
    */
    for (int j = 0; j < 50; j++) {
      if (fabs(d[j] - d_) < 2.0) {
        double dist = distance(s[j], d[j], (s_ + v*(j+1)*0.02), d_);
        min_distance = min(min_distance, dist);
      }
    }
  }
  cout << "distance = " << min_distance << endl;
  result *= exp(-min_distance);
  return result * COLLISION;
}

double Trajectory::inefficiency_cost() const {
  return fabs(final_v - (SPEED_LIMIT-SPEED_LIMIT_BUFFER)/2.24)*EFFICIENCY;
}

double Trajectory::overspeeding_cost() const {
  return sgn(final_v - SPEED_LIMIT)*DANGER;
}
