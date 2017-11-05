#include "cost_functions.h"
#include "helper_functions.h"
#include "spline.h"
#include <vector>
#include <cfloat>
#include <algorithm>
#include <math.h>

using namespace std;

static bool abs_compare(double a, double b)
{
    return (fabs(a) < fabs(b));
}

double Trajectory::cost (vector<vector<double>> sensor_fusion) const {
  double result = 0.0;
  double cost = collision_cost(sensor_fusion);
  cout << "COLLISION = " << cost << endl;
  result += cost*COLLISION;
  cost = inefficiency_cost();
  cout << "INEFFICIENCY = " << cost << endl;
  result += cost*EFFICIENCY;
  cost = acceleration_cost();
  cout << "ACCELERATION GOAL = " << cost << endl;
  result += cost*PROJECT_GOAL;
  cost = jerk_cost();
  cout << "JERK GOAL = " << cost << endl;
  result += cost*PROJECT_GOAL;
  cost = overspeeding_cost();
  cout << "OVERSPEEDING = " << cost << endl;
  result += cost*PROJECT_GOAL;
  return result;
}

double Trajectory::acceleration_cost() const{
  double acc = sqrt(pow(max_acceleration_s, 2.0) + pow(max_acceleration_d, 2.0));
  if (acc >= MAX_ACCELERATION) return 1.0;
  return 0.0;
}

double Trajectory::jerk_cost() const {
  double jerk = sqrt(pow(max_jerk_s, 2.0) + pow(max_jerk_d, 2.0));
  if (jerk >= MAX_JERK) return 1.0;
  return 0.0;
}

double Trajectory::comfort_cost() const{
  double result = 0.0;
  
  result += (exp(max_acceleration_s/(0.9*MAX_ACCELERATION)) - 1.0)*COMFORT;
  result += (exp(max_jerk_s/(0.9*MAX_JERK)) - 1.0)*COMFORT;
  
  result += (exp(max_acceleration_d/(0.9*MAX_ACCELERATION)) - 1.0)*COMFORT;
  result += (exp(max_jerk_d/(0.9*MAX_JERK)) - 1.0)*COMFORT;
  return result;
}

double Trajectory::collision_cost(vector<vector<double>> sensor_fusion) const {
  double result = 1.0;
  if (lane < 0 or lane >= 3) {
    result += 1;
  }
  cout << endl;
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
    int lane_ = (int)(d_/4.0);
    /*
    for (int j = 0; j < x.size(); j++) {
      double current_x = x_ + vx*j*0.02;
      double current_y = y_ + vy*j*0.02;
      vector<double> sd = getFrenet(current_x, )
    }
    */
    //cout << "vehicle : " << id << " " << (start_s - s_) << " " << (start_d - d_) << " " << v;
    double curr_distance = 1000.0;
    int help_j = -1;
    for (int j = 0; j < min(50ul, s.size()); j++) {
      int lane_j = (int)(d[j]/4.0);
      if (lane_ == lane_j || lane_ == lane || fabs(d[j] - d_) < 2.0) {
        double dist = fabs(s[j] - (s_ + v*(j+1)*0.02));
        //double dist = distance(s[j], d[j], (s_ + v*(j+1)*0.02), d_);
        if (curr_distance > dist) {
          //cout << "> s distance = " << dist << ", d distance = " << fabs(d[j] - d_) << " at " << ((j+1)*0.02) << " s " << endl; 
          help_j = j;
        }
        curr_distance = min(curr_distance, dist);
      }
    }
    //cout << " distance to this vehicle = " << curr_distance  << "(" << help_j << ")" << endl;
    min_distance = min(min_distance, curr_distance);
  }
  cout << "distance = " << min_distance << endl;
  if (min_distance < MIN_DISTANCE) {
    return 1.0;
  }
  if (min_distance < RECOMMENDED_DISTANCE) {
    return 0.3;
  }
  return 0.0;
}

double Trajectory::inefficiency_cost() const {
  double target = miph_to_mps(SPEED_LIMIT-SPEED_LIMIT_BUFFER);
  return fabs(target - avg_speed)/target;
}

double Trajectory::overspeeding_cost() const {
  double extra = max(max_speed, max_velocity_s) - miph_to_mps(SPEED_LIMIT - SPEED_LIMIT_BUFFER);
  if (extra <= 0.0) return 0.0;
  return 1.0;
}

Trajectory Trajectory::create_CL_trajectory(CarLocation current, Target target, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y, double T) {
  Trajectory result;
  if (target.lane < 0 || target.lane > 2) {
    return ImpossibleTrajectory();
  }
  int prev_size = current.previous_path_x.size();

  vector<double> ptsx, ptsy;
  double ref_x = current.car_x;
  double ref_y = current.car_y;

  double ref_yaw = deg2rad(current.car_yaw);

  double prev_car_x = ref_x - cos(ref_yaw);
  double prev_car_y = ref_y - sin(ref_yaw);
  

  ptsx.push_back(prev_car_x);
  ptsx.push_back(ref_x);

  ptsy.push_back(prev_car_y);
  ptsy.push_back(ref_y);

  vector<double> s_start = {current.car_s, current.car_speed, current.car_acceleration};
  vector<double> s_final = {target.s, target.v, target.a};
  vector<double> s_coeff = jmt(s_start, s_final, T);
  
  vector<double> s_v = dif(s_coeff);
  vector<double> s_a = dif(s_v);
  vector<double> s_j = dif(s_a);
  cout << "T = " << T << endl;
  cout << "s_start = [" << eval(s_coeff, 0.0) << ", " << eval(s_v, 0.0) << ", " << eval(s_a, 0.0) << ']' << endl;
  cout << "s_end = [" << eval(s_coeff, T) << ", " << eval(s_v, T) << ", " << eval(s_a, T) << ']' << endl;

  result.max_acceleration_s = 0.0;
  result.max_jerk_s = 0.0;
  result.max_velocity_s = 0.0;

  for (double t = 0.0; t <= T; t += DELTA_T) {
    result.max_velocity_s = max(result.max_velocity_s, fabs(eval(s_v, t)));
    result.max_acceleration_s = max(result.max_acceleration_s, fabs(eval(s_a, t)));
    result.max_jerk_s = max(result.max_jerk_s, fabs(eval(s_j, t)));
  }

  /*
  for (int i = 0; i < s_coeff.size(); i++) {
    cout << s_coeff[i] << " ";
  }
  cout << endl;
  */

  vector<double> d_start = {current.car_d, 0.0, 0.0};
  vector<double> d_final = {4.0*target.lane+2.0, 0.0, 0.0};
  vector<double> d_coeff = jmt(d_start, d_final, T);

  vector<double> d_v = dif(d_coeff);
  vector<double> d_a = dif(d_v);
  vector<double> d_j = dif(d_a);
  cout << "d_start = [" << eval(d_coeff, 0.0) << ", " << eval(d_v, 0.0) << ", " << eval(d_a, 0.0) << ']' << endl;
  cout << "d_end = [" << eval(d_coeff, T) << ", " << eval(d_v, T) << ", " << eval(d_a, T) << ']' << endl;

  result.max_acceleration_d = 0.0;
  result.max_jerk_d = 0.0;
  result.max_velocity_d = 0.0;

  for (double t = 0.0; t <= T; t += DELTA_T) {
    result.max_velocity_d = max(result.max_velocity_d, fabs(eval(d_v, t)));
    result.max_acceleration_d = max(result.max_acceleration_d, fabs(eval(d_a, t)));
    result.max_jerk_d = max(result.max_jerk_d, fabs(eval(d_j, t)));
  }

  //cout << "> " << current.car_s << " + " << vel_m_per_s << " * X" << endl; 
  
  for (double t = 0.5; t <= T + 0.1; t += 0.5) {
    double s_val = eval(s_coeff, t);
    double d_val = eval(d_coeff, t);
    //cout << ">> " << s_val << " <o> " << d_val <<  " <<"<< endl;
    vector<double> next_wp = getXY(s_val, d_val, maps_s, maps_x, maps_y);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);
  }

  //cout << "car_yaw = " << current.car_yaw <<  " degrees"<< endl;
  /*
  for (int i = 0; i < ptsx.size(); i++) {
    cout << ptsx[i] << " " << ptsy[i] << endl;
  }
  cout << endl;
  */

  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  
  /*
  for (int i = 0; i < ptsx.size(); i++) {
    cout << ptsx[i] << " " << ptsy[i] << endl;
  }
  cout << endl;
  */

  for (int i = 0; i < ptsx.size() - 1; i++) {
    if (ptsx[i] >= ptsx[i+1]) {
      return ImpossibleTrajectory();
    }
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);

  double trajectory_len = 3.0; // in seconds
  double target_x = eval(s_v, 0.0)*trajectory_len;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for (int i = 1; i <= (int)(trajectory_len*50); i++) {
    double current_speed = eval(s_v, 0.02*i);
    double N = (target_dist/(.02*current_speed));
    double x_point = x_add_on + (target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    result.x.push_back(x_point);
    result.y.push_back(y_point);
    //vector<double> sd = getFrenet(x_point, y_point, deg2rad(current.car_yaw), maps_x, maps_y);
    //result.s.push_back(sd[0]);
    //result.d.push_back(sd[1]);
    result.s.push_back(eval(s_coeff, 0.02*i));
    result.d.push_back(eval(d_coeff, 0.02*i));
  }

  /*
  vector<double> s_v_, s_a_, s_j_;
  for (int i = 1; i < result.s.size(); i++) {
    s_v_.push_back((result.s[i] - result.s[i-1])/0.02);
  }
  for (int i = 1; i < s_v_.size(); i++) {
    s_a_.push_back((s_v_[i] - s_v_[i-1])/0.02);
  }
  for (int i = 1; i < s_a_.size(); i++) {
    s_j_.push_back((s_a_[i] - s_a_[i-1])/0.02);
  }


  result.max_velocity_s = *std::max_element(s_v_.begin(), s_v_.end(), abs_compare);
  result.max_acceleration_s = *std::max_element(s_a_.begin(), s_a_.end(), abs_compare);
  result.max_jerk_s = *std::max_element(s_j_.begin(), s_j_.end(), abs_compare);
  */
  /*
  double temp_max_dist = 0.0;
  for (int i = 0; i < result.s.size(); i++) {
    temp_max_dist = max(temp_max_dist, result.s[i] - eval(s_coeff, 0.02*(i+1)));
    if (i % 10 == 0) {
      cout << result.s[i] << "(" << result.x[i] << ")" << " -+- " << eval(s_coeff, 0.02*(i+1)) << " in " << 0.02*(i+1) << endl;
    }
  }
  cout << "max error of smoothing = " << temp_max_dist << endl;
  */

  result.start_s = result.s[0];
  result.final_s = result.s[result.s.size()-1];
  result.start_d = result.d[0];
  result.final_d = result.d[result.d.size()-1];
  result.final_v = eval(s_v, trajectory_len*50*0.02);

  result.lane_change = current.car_lane - target.lane;
  result.lane = target.lane;

  result.calcAvgSpeed();
  
  return result;
}

Trajectory Trajectory::create_trajectory(CarLocation current, Target target, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
  Trajectory result;
  if (target.lane < 0 || target.lane > 2) {
    result.final_d = target.lane*4+2;
    return result;
  }
  int prev_size = current.previous_path_x.size();
  vector<double> ptsx, ptsy;
  double ref_x = current.car_x;
  double ref_y = current.car_y;
  //cout << "yaw = " << current.car_yaw << endl;
  double ref_yaw = deg2rad(current.car_yaw);

  if (prev_size < 2) {
    double prev_car_x = current.car_x - cos(ref_yaw);
    double prev_car_y = current.car_y - sin(ref_yaw);

    ptsx.push_back(prev_car_x);
    ptsx.push_back(current.car_x);

    ptsy.push_back(prev_car_y);
    ptsy.push_back(current.car_y);
  }
  else {
    ref_x = current.previous_path_x[prev_size - 1];
    ref_y = current.previous_path_y[prev_size - 1];

    double ref_x_prev = current.previous_path_x[prev_size-2];
    double ref_y_prev = current.previous_path_y[prev_size-2];

    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

    ptsx.push_back(ref_x_prev);
    ptsx.push_back(ref_x);

    ptsy.push_back(ref_y_prev);
    ptsy.push_back(ref_y);
  }
  //cout << "ref_yaw = " << (ref_yaw * 180.0 / M_PI) << endl;
  
  vector<double> ref_sd = getFrenet(ref_x, ref_y, current.car_yaw, maps_x, maps_y);
  double ref_s = ref_sd[0];
  double ref_d = ref_sd[1];

  double vel_m_per_s = max(1.0, target.v);
  //cout << "> " << ref_s << " + " << vel_m_per_s << " * X" << endl; 
  vector<double> next_wp0 = getXY(ref_s + vel_m_per_s*2.0, (2.0+4.0*target.lane), maps_s, maps_x, maps_y);
  vector<double> next_wp1 = getXY(ref_s + vel_m_per_s*3.0, (2.0+4.0*target.lane), maps_s, maps_x, maps_y);
  vector<double> next_wp2 = getXY(ref_s + vel_m_per_s*5.0, (2.0*4.0*target.lane), maps_s, maps_x, maps_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  /*
  for (int i = 0; i < ptsx.size(); i++) {
    cout << ptsx[i] << ", " << ptsy[i] << endl;
  }
  cout << endl;
  */

  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  
  
  /*
  for (int i = 0; i < ptsx.size(); i++) {
    cout << ptsx[i] << ", " << ptsy[i] << endl;
  }
  */

  for (int i = 0; i < ptsx.size() - 1; i++) {
    if (ptsx[i] >= ptsx[i+1]) {
      return ImpossibleTrajectory();
    }
  }

  tk::spline s;
  s.set_points(ptsx, ptsy);
  for (int i = 0; i < current.previous_path_x.size(); i++) {
    result.x.push_back(current.previous_path_x[i]);
    result.y.push_back(current.previous_path_y[i]);
    vector<double> sd = getFrenet(current.previous_path_x[i], current.previous_path_y[i], current.car_yaw, maps_x, maps_y);
    result.s.push_back(sd[0]);
    result.d.push_back(sd[1]);
  }

  double target_x = 40.0;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for (int i = 1; i <= 50-prev_size; i++) {
    double N = (target_dist/(.02*target.v));
    double x_point = x_add_on + (target_x)/N;
    double y_point = s(x_point);

    x_add_on = x_point;

    double x_ref = x_point;
    double y_ref = y_point;

    x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
    y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

    x_point += ref_x;
    y_point += ref_y;

    result.x.push_back(x_point);
    result.y.push_back(y_point);
    //vector<double> sd = getFrenet(x_point, y_point, current.car_yaw, maps_x, maps_y);
    //result.s.push_back(sd[0]);
    //result.d.push_back(sd[1]);
    result.s.push_back(current.car_s + x_add_on);
    result.d.push_back(current.car_d + s(x_add_on));
  }
  result.start_s = result.s[0];
  result.final_s = result.s[result.s.size()-1];
  result.start_d = result.d[0];
  result.final_d = result.d[result.d.size()-1];
  result.final_v = target.v;


  result.lane_change = current.car_lane - target.lane;
  result.lane = target.lane;
  
  result.calcAvgSpeed();

  result.calcMaxSpeed();

  return result;
}

void Trajectory::calcMaxSpeed() {
  double temp_max = 0.0;
  for (int i = 1; i < x.size(); i++) {
    temp_max = max(temp_max, distance(x[i], y[i], x[i-1], y[i-1]));
  }
  max_speed = temp_max/0.02;
}

void Trajectory::calcAvgSpeed() {
  double sum = 0.0;
  for (int i = 1; i < x.size(); i++) {
    sum += distance(x[i], y[i], x[i-1], y[i-1]);
  }
  sum /= (x.size()-1);
  avg_speed = sum/0.02;
}

void Trajectory::printToStdout() const {
  cout << "-------------------------------"  << endl;
  cout << "| final_d = " << final_d << endl;
  cout << "| final_s = " << final_s << endl;
  cout << "| final_v = " << final_v << endl;
  cout << "| lane = " << lane << " lane_change = " << lane_change << endl;
  cout << "| avg speed = " << avg_speed <<  " max speed = " << max_speed << endl;
  cout << "|- s: " << endl;
  cout << "|   max_velocity = " << max_velocity_s << endl;
  cout << "|   max_acceleration = " << max_acceleration_s << endl;
  cout << "|   max_jerk = " << max_jerk_s << endl;
  cout << "|- d: " << endl;
  cout << "|   max_velocity = " << max_velocity_d << endl;
  cout << "|   max_acceleration = " << max_acceleration_d << endl;
  cout << "|   max_jerk = " << max_jerk_d << endl;
  cout << "-------------------------------"  << endl;
}

double ImpossibleTrajectory::cost(std::vector<std::vector<double>> sensor_fusion) const {
  return DBL_MAX;
}

void ImpossibleTrajectory::printToStdout() const {
  cout << "ImpossibleTrajectory" << endl;
}

