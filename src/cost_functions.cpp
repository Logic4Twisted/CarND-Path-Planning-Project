#include "cost_functions.h"
#include "spline.h"
#include <vector>

using namespace std;

double Trajectory::cost (vector<vector<double>> sensor_fusion) const {
  double result = 0.0;
  double cost = collision_cost(sensor_fusion);
  cout << "COLLISION = " << cost << endl;
  result += cost;
  cost = inefficiency_cost();
  cout << "INEFFICIENCY = " << cost << endl;
  result += cost;
  cost = comfort_cost();
  cout << "COMFORT = " << cost << endl;
  result += cost;
  cost = overspeeding_cost();
  cout << "OVERSPEEDING = " << cost << endl;
  result += cost;
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
    cout << "vehicle : " << id << " " << s_ << " " << d_ << " " << v;
    double curr_distance = 1000.0;
    int help_j = -1;
    for (int j = 0; j < s.size(); j++) {
      int lane_j = int(d[j]/4.0);
      if (lane_ == lane_j || lane_ == lane || fabs(d[j] - d_) < 2.0) {
        double dist = fabs(s[j] - (s_ + v*(j+1)*0.02));
        //double dist = distance(s[j], d[j], (s_ + v*(j+1)*0.02), d_);
        if (curr_distance > dist) {
          help_j = j;
        }
        curr_distance = min(curr_distance, dist);
      }
    }
    cout << " distance to this vehicle = " << curr_distance  << "(" << help_j << ")" << endl;
    min_distance = min(min_distance, curr_distance);
  }
  cout << "distance = " << min_distance << endl;
  result *= exp(-min_distance/5.0);
  return result * COLLISION;
}

double Trajectory::inefficiency_cost() const {
  return fabs(final_v - (SPEED_LIMIT-SPEED_LIMIT_BUFFER)/2.24)*EFFICIENCY;
}

double Trajectory::overspeeding_cost() const {
  return sgn(final_v - SPEED_LIMIT)*DANGER;
}

Trajectory Trajectory::create_CL_trajectory(CarLocation current, Target target, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y) {
  Trajectory result;
  if (target.lane < 0 || target.lane > 2) {
    result.final_d = target.lane*4+2;
    return result;
  }
  double T = 3.0; // 3 seconds to change lane
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
  vector<double> s_final = {target.s, target.v, current.car_acceleration};
  vector<double> s_coeff = jmt(s_start, s_final, T);
  
  cout << "s 0 < " << eval(s_coeff, 0.0) << ", " << eval(dif(s_coeff), 0.0) << ", " << eval(dif(dif(s_coeff)), 0.0) << endl;
  cout << "s T > " << eval(s_coeff, T) << ", " << eval(dif(s_coeff), T) << ", " << eval(dif(dif(s_coeff)), T) << endl;

  for (int i = 0; i < s_coeff.size(); i++) {
    cout << s_coeff[i] << " ";
  }
  cout << endl;

  vector<double> d_start = {current.car_d, 0.0, 0.0};
  vector<double> d_final = {4.0*target.lane+2.0, 0.0, 0.0};
  vector<double> d_coeff = jmt(d_start, d_final, T);

  cout << "d 0 < " << eval(d_coeff, 0.0) << ", " << eval(dif(d_coeff), 0.0) << ", " << eval(dif(dif(d_coeff)), 0.0) << endl;
  cout << "d T > " << eval(d_coeff, T) << ", " << eval(dif(d_coeff), T) << ", " << eval(dif(dif(d_coeff)), T) << endl;

  //cout << "> " << current.car_s << " + " << vel_m_per_s << " * X" << endl; 
  
  for (double t = 1.0; t <= T + 0.1; t += 1.0) {
    double s_val = eval(s_coeff, t);
    double d_val = eval(d_coeff, t);
    cout << ">> " << s_val << " <> " << d_val <<  " <<"<< endl;
    vector<double> next_wp = getXY(s_val, d_val, maps_s, maps_x, maps_y);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);
  }

  cout << "car_yaw = " << current.car_yaw <<  " degrees"<< endl;
  for (int i = 0; i < ptsx.size(); i++) {
    cout << ptsx[i] << " " << ptsy[i] << endl;
  }
  cout << endl;

  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  tk::spline s;

  for (int i = 0; i < ptsx.size(); i++) {
    cout << ptsx[i] << " " << ptsy[i] << endl;
  }
  cout << endl;

  s.set_points(ptsx, ptsy);

  double target_x = current.car_speed*T + current.car_acceleration*pow(T, 2.0)/2;
  double target_y = s(target_x);
  double target_dist = sqrt(target_x*target_x + target_y*target_y);

  double x_add_on = 0;

  for (int i = 1; i <= 150; i++) {
    double current_speed = eval(dif(s_coeff), 0.02*i);
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
    vector<double> sd = getFrenet(x_point, y_point, current.car_yaw, maps_x, maps_y);
    result.s.push_back(sd[0]);
    result.d.push_back(sd[1]);
  }

  result.start_s = result.s[0];
  result.final_s = result.s[result.s.size()-1];
  result.start_d = result.d[0];
  result.final_d = result.d[result.d.size()-1];
  result.final_v = eval(dif(s_coeff), T*50*0.02);
  cout << "final_d = " << result.final_d << endl;
  cout << "final_s = " << result.final_s << endl;
  cout << "final_v = " << result.final_v << endl;
  //result.final_v = (result.s[result.s.size()-1] - result.s[result.s.size()-2])/0.02;

  result.lane_change = current.car_lane - target.lane;
  result.lane = target.lane;

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
  cout << "ref_yaw = " << (ref_yaw * 180.0 / M_PI) << endl;
  
  double vel_m_per_s = max(1.0, target.v);
  cout << "> " << current.car_s << " + " << vel_m_per_s << " * X" << endl; 
  vector<double> next_wp0 = getXY(current.car_s+vel_m_per_s*3, (2+4*target.lane), maps_s, maps_x, maps_y);
  vector<double> next_wp1 = getXY(current.car_s+vel_m_per_s*4, (2+4*target.lane), maps_s, maps_x, maps_y);
  vector<double> next_wp2 = getXY(current.car_s+vel_m_per_s*6, (2*4*target.lane), maps_s, maps_x, maps_y);

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); i++) {
    cout << ptsx[i] << ", " << ptsy[i] << endl;
  }
  cout << endl;

  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

  tk::spline s;
  

  for (int i = 0; i < ptsx.size(); i++) {
    cout << ptsx[i] << ", " << ptsy[i] << endl;
  }

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
    vector<double> sd = getFrenet(x_point, y_point, current.car_yaw, maps_x, maps_y);
    result.s.push_back(sd[0]);
    result.d.push_back(sd[1]);
  }
  result.start_s = result.s[0];
  result.final_s = result.s[result.s.size()-1];
  result.start_d = result.d[0];
  result.final_d = result.d[result.d.size()-1];
  result.final_v = target.v;

  cout << "final_d = " << result.final_d << endl;
  cout << "final_s = " << result.final_s << endl;
  cout << "final_v = " << result.final_v << endl;

  result.lane_change = current.car_lane - target.lane;
  result.lane = target.lane;
  

  return result;
}
