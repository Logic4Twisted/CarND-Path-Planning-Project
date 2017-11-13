#include "cost_functions.h"
#include "helper_functions.h"
#include "spline.h"
#include <vector>
#include <cfloat>
#include <algorithm>
#include <math.h>
#include <unordered_map>
#include <random>

using namespace std;

static bool abs_compare(double a, double b)
{
    return (fabs(a) < fabs(b));
}

double Trajectory::cost (vector<vector<double>> sensor_fusion) const {
  double result = 0.0;

  double cost = collision_cost(sensor_fusion);
  //cout << "COLLISION = " << cost << endl;
  result += cost*COLLISION;
  
  cost = inefficiency_cost();
  //cout << "INEFFICIENCY = " << cost << endl;
  result += cost*EFFICIENCY;
  
  cost = acceleration_cost();
  //cout << "ACCELERATION GOAL = " << cost << endl;
  result += cost*PROJECT_GOAL;
  
  cost = jerk_cost();
  //cout << "JERK GOAL = " << cost << endl;
  result += cost*PROJECT_GOAL;
  
  cost = overspeeding_cost();
  //cout << "OVERSPEEDING = " << cost << endl;
  result += cost*PROJECT_GOAL;

  cost = comfort_cost();
  //cout << "COMFORT = " << cost << endl;
  result += cost*COMFORT;

  cost = change_lane_cost();
  //cout << "CHANGE LANE COST = " << cost << endl;
  result += cost*LANE_CHANGE; 

  cost = cost_outside_lanes();
  //cout << "DRIVING OUTSIDE LANES = " << cost << endl;
  result += cost*TRAFIC_LAWS;
  
  return result;
}

double Trajectory::cost_outside_lanes() const {
  if (min_legal_distance > 1.5) return 0.0;
  if (min_legal_distance < 0.0) return 1.0;
  return (1.5-min_legal_distance)/1.5;
}

double Trajectory::change_lane_cost() const{
  return fabs(d.front() - d.back())/4.0;
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
  result += (exp(max_jerk_d/(10.0)) - 1.0);
  return result;
}

double Trajectory::collision_cost(vector<vector<double>> sensor_fusion) const {
  double result = 1.0;
  if (lane < 0 or lane >= 3) {
    result += 1;
  }
  int vehicle_id = -1;
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
    int lane_ = (int)floor(d_/4.0);

    double curr_distance = 1000.0;
    int help_j = -1;
    for (int j = 0; j < s.size(); j++) {
      int lane_j = (int)floor(d[j]/4.0);
      if (lane_ == lane_j || lane_ == lane || fabs(d[j] - d_) < 2.0) {
        //double dist = fabs(s[j] - (s_ + v*(j+1)*0.02));
        double dist = distance(s[j], d[j], (s_ + v*(j+1)*0.02), d_);
        if (curr_distance > dist) {
          help_j = j;
        }
        curr_distance = min(curr_distance, dist);
      }
    }
    if (curr_distance < min_distance) {
      vehicle_id = id;
    }
    min_distance = min(min_distance, curr_distance);
  }
  //cout << "distance = " << min_distance << "   vehicle_id = " << vehicle_id << endl;
  if (min_distance < MIN_DISTANCE) {
    return 1.0;
  }
  if (min_distance < RECOMMENDED_DISTANCE) {
    double x = (RECOMMENDED_DISTANCE - min_distance)/(RECOMMENDED_DISTANCE - MIN_DISTANCE);
    return x*x;
  }
  return 0.0;
}

double Trajectory::inefficiency_cost() const {
  //double target = miph_to_mps(SPEED_LIMIT-SPEED_LIMIT_BUFFER);
  double diff = avg_velocity_s - start_v;
  return 1.0/(1.0 + exp(diff));
}

double Trajectory::overspeeding_cost() const {
  double extra = max_velocity_s - miph_to_mps(SPEED_LIMIT - SPEED_LIMIT_BUFFER);
  if (extra <= 0.0) return 0.0;
  return 1.0;
}


Trajectory Trajectory::create_trajectory(CarLocation current, Target target, HighwayMap map) {
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
  
  vector<double> ref_sd = map.getFrenet(ref_x, ref_y, current.car_yaw);
  double ref_s = ref_sd[0];
  double ref_d = ref_sd[1];

  double vel_m_per_s = max(1.0, target.v);
  //cout << "> " << ref_s << " + " << vel_m_per_s << " * X" << endl; 
  vector<double> next_wp0 = map.getXY(ref_s + vel_m_per_s*2.0, (2.0+4.0*target.lane));
  vector<double> next_wp1 = map.getXY(ref_s + vel_m_per_s*3.0, (2.0+4.0*target.lane));
  vector<double> next_wp2 = map.getXY(ref_s + vel_m_per_s*5.0, (2.0*4.0*target.lane));

  ptsx.push_back(next_wp0[0]);
  ptsx.push_back(next_wp1[0]);
  ptsx.push_back(next_wp2[0]);

  ptsy.push_back(next_wp0[1]);
  ptsy.push_back(next_wp1[1]);
  ptsy.push_back(next_wp2[1]);

  for (int i = 0; i < ptsx.size(); i++) {
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;

    ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
    ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
  }

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
    vector<double> sd = map.getFrenet(current.previous_path_x[i], current.previous_path_y[i], current.car_yaw);
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
  
  //result.calcAvgSpeed();

  //result.calcMaxSpeed();

  return result;
}

unordered_map<string, Trajectory> Trajectory::generate(Trajectory previous, CarLocation current, HighwayMap map, double T) {
  unordered_map<string, Trajectory> result;

  while (!previous.empty() && (current.car_s > previous.s[0] || (!previous.empty() && (previous.s[0] - current.car_s) > (TRACK_LEN - 20.0)))) {
    previous.pop_front();
  }

  vector<vector<double>> reuse_previous;
  while (!previous.s.empty() && reuse_previous.size() < 10) {
    reuse_previous.push_back(previous.front());
    previous.pop_front();
  }

  double start_s = current.car_s;
  double start_sv = current.car_speed;
  double start_sa = current.car_acceleration;
  if (!reuse_previous.empty()) {
    start_s = reuse_previous.back()[0];
    start_sv = reuse_previous.back()[1];
    start_sa = reuse_previous.back()[2];
  }
  vector<double> s_start = {start_s, start_sv, start_sa};

  double start_d = current.car_d;
  double start_dv = 0.0;
  double start_da = 0.0;
  if (!reuse_previous.empty()) {
    start_d = reuse_previous.back()[3];
    start_dv = reuse_previous.back()[4];
    start_da = reuse_previous.back()[5];
  }
  vector<double> d_start = {start_d, start_dv, start_da};


  vector<double> dif_acc = {-6.0, -4.0, -3.0, -2.0, -1.5, -1.0, -0.8, -0.6, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.6, 0.8, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0};
  vector<int> dif_lane = {0, 1, -1};
  for (int i = 0; i < dif_acc.size(); i++) {
    for (int j = 0; j < dif_lane.size(); j++) {

      Trajectory t;
      string identifier = "Tr_" + to_string(result.size());

      // reuse some previous points
      for (int i = 0; i < reuse_previous.size(); i++) {
        t.push_back(reuse_previous[i]);
      }



      // calculate s values
      double a1 = dif_acc[i];

      double finish_a = start_sa + a1*T;
      double finish_v = start_sv + start_sa*T + a1*T*T/2.0;
      double finish_s = start_s + start_sv*T + start_sa*T*T/2.0 + a1*T*T*T/6.0;
      if (finish_s <= start_s) {
        //cout << "Going backwards - abort this trajectory " << endl;
        continue;
      }
      vector<double> s_final = {finish_s, finish_v, finish_a};
      vector<double> s_coeff = jmt(s_start, s_final, T);



      // calculate d values
      int a2 = dif_lane[j];
      t.lane = current.car_lane;
      t.lane_change = a2;
      if (t.lane_change == -1.0) {
        identifier += " CL";
      }
      else if (t.lane_change == 1.0) {
        identifier += " CR";
      }
      t.target_lane = t.lane + t.lane_change;
      //cout << endl << "--> " << identifier << endl;

      if (t.target_lane < 0 || t.target_lane > 2) {
        continue;
      }

      double finish_d = 4.0*t.target_lane + 2.0;
      vector<double> d_final = {finish_d, 0.0, 0.0};
      vector<double> d_coeff = jmt(d_start, d_final, T);


      // calculate stuff for trajetories
      t.calculate_waypoints(s_coeff, d_coeff, T);
      t.calculateStats();

      result[identifier] = t;
    }
  }

  return result;
}

unordered_map<string, Trajectory> Trajectory::generate_random(Trajectory previous, CarLocation current, HighwayMap map, int N_to_generate, double T) {
  unordered_map<string, Trajectory> result;

  std::default_random_engine de(time(0));
  while (!previous.empty() && (current.car_s > previous.s[0] || (previous.s[0] - current.car_s) > (TRACK_LEN - 20.0))) {
    previous.pop_front();
  }

  vector<vector<double>> reuse_previous;
  while (!previous.s.empty() && reuse_previous.size() < 10) {
    reuse_previous.push_back(previous.front());
    previous.pop_front();
  }

  double start_s = current.car_s;
  double start_sv = current.car_speed;
  double start_sa = current.car_acceleration;
  if (!reuse_previous.empty()) {
    start_s = reuse_previous.back()[0];
    start_sv = reuse_previous.back()[1];
    start_sa = reuse_previous.back()[2];
  }
  vector<double> s_start = {start_s, start_sv, start_sa};

  double start_d = current.car_d;
  double start_dv = 0.0;
  double start_da = 0.0;
  if (!reuse_previous.empty()) {
    start_d = reuse_previous.back()[3];
    start_dv = reuse_previous.back()[4];
    start_da = reuse_previous.back()[5];
  }
  vector<double> d_start = {start_d, start_dv, start_da};

  int try_counter = 0;
  while (result.size() < N_to_generate && try_counter < 2*N_to_generate) {
    try_counter++;

    Trajectory t;
    string identifier = "Tr_" + to_string(result.size());
    //cout << endl << "--> " << identifier << endl;



    // reuse some previous points
    for (int i = 0; i < reuse_previous.size(); i++) {
      t.push_back(reuse_previous[i]);
    }



    // calculate s values
    std::normal_distribution<> dist(0.0, 1.0);
    double a1 = dist(de);
    //cout << "Random variable = " << a1 << endl;

    double finish_a = start_sa + a1*T;
    double finish_v = start_sv + start_sa*T + a1*T*T/2.0;
    double finish_s = start_s + start_sv*T + start_sa*T*T/2.0 + a1*T*T*T/6.0;
    if (finish_s <= start_s) {
      //cout << "Going backwards - abort this trajectory " << endl;
      continue;
    }
    vector<double> s_final = {finish_s, finish_v, finish_a};
    vector<double> s_coeff = jmt(s_start, s_final, T);



    // calculate d values
    double a2 = dist(de);
    t.lane = current.car_lane;
    t.lane_change = 0;
    if (a2 < -1.0) {
      if (t.lane > 0) {
        t.lane_change = -1;
        identifier += " CL";
      }
      else if (t.lane < 2) {
        t.lane_change = 1;
        identifier += " CR";
      }
      
    }
    else if (a2 > 1.0 && t.lane < 2) {
      if (t.lane < 2) {
        t.lane_change = 1;
        identifier += " CR";
      }
      else if (t.lane > 0) {
        t.lane_change = -1;
        identifier += " CL";
      }
      
    }
    t.target_lane = t.lane + t.lane_change;

    double finish_d = 4.0*t.target_lane + 2.0;

    //cout << "start_d = " << start_d << " finish_d = " << finish_d << endl;
    vector<double> d_final = {finish_d, 0.0, 0.0};
    vector<double> d_coeff = jmt(d_start, d_final, T);


    // calculate stuff for trajetories
    t.calculate_waypoints(s_coeff, d_coeff, T);
    t.calculateStats();

    result[identifier] = t;
  }

  return result;
}

Trajectory Trajectory::reuseTrajectory(Trajectory previous, CarLocation current, HighwayMap map) {

  Trajectory result;
  while (!previous.s.empty() && current.car_s > previous.s[0]) {
    previous.pop_front();
  }

  vector<vector<double>> reuse_previous;
  while (!previous.empty()) {
    result.push_back(previous.front());
    previous.pop_front();
  }
  result.target_lane = previous.target_lane;
  result.lane_change = previous.lane_change;
  result.lane = previous.lane;
  result.calculateStats();
  return result;
}

bool Trajectory::changingLanes() {
  return lane_change != 0;
}

int Trajectory::getTargetLane() {
  return target_lane;
}

void Trajectory::calculate_waypoints(vector<double> s_coeff, vector<double> d_coeff, double T) {
  vector<double> s_v = dif(s_coeff);
  vector<double> s_a = dif(s_v);
  vector<double> s_j = dif(s_a);
  //cout << "T = " << T << endl;
  //for (int i = 0; i < s_coeff.size(); i++) {
  //  cout << s_coeff[i] << " ";
  //}
  //cout << endl;
  //cout << "s_start = [" << eval(s_coeff, 0.0) << ", " << eval(s_v, 0.0) << ", " << eval(s_a, 0.0) << ']' << endl;
  //cout << "s_end = [" << eval(s_coeff, T) << ", " << eval(s_v, T) << ", " << eval(s_a, T) << ']' << endl;

  vector<double> d_v = dif(d_coeff);
  vector<double> d_a = dif(d_v);
  vector<double> d_j = dif(d_a);
  //cout << "d_start = [" << eval(d_coeff, 0.0) << ", " << eval(d_v, 0.0) << ", " << eval(d_a, 0.0) << ']' << endl;
  //cout << "d_end = [" << eval(d_coeff, T) << ", " << eval(d_v, T) << ", " << eval(d_a, T) << ']' << endl;

  max_acceleration_s = 0.0;
  max_jerk_s = 0.0;
  max_velocity_s = 0.0;
  min_velocity_s = 0.0;

  max_jerk_d = 0.0;
  max_acceleration_d = 0.0;
  max_velocity_d = 0.0;

  min_legal_distance = 6.0;

  for (double dt = 0.02; dt <= T; dt += 0.02) {
    double s_ = fmod(eval(s_coeff, dt), TRACK_LEN);
    double sv_ = eval(s_v, dt);
    double sa_ = eval(s_a, dt);
    double sj_ = eval(s_j, dt);

    double d_ = eval(d_coeff, dt);
    double dv_ = eval(d_v, dt);
    double da_ = eval(d_a, dt);
    double dj_ = eval(d_j, dt);
      
    s.push_back(s_);
    sv.push_back(sv_);
    sa.push_back(sa_);

    d.push_back(d_);
    dv.push_back(dv_);
    da.push_back(da_);

    max_jerk_s = max(max_jerk_s, sj_);
    max_velocity_s = max(max_velocity_s, sv_);
    max_acceleration_s = max(max_acceleration_s, sa_);
    min_velocity_s = min(min_velocity_s, sv_);

    max_velocity_d = max(fabs(max_velocity_d), dv_);
    max_acceleration_d = max(max_acceleration_d, da_);
    max_jerk_s = max(max_jerk_s, dj_);

    min_legal_distance = min(min_legal_distance, min(12.0 - d_, d_));
  }

  start_v = sv.front();
  final_v = sv.back();
}

bool Trajectory::empty() {
  return s.empty();
}

vector<double> Trajectory::front() {
  vector<double> result;
  result.push_back(s.front());
  result.push_back(sv.front());
  result.push_back(sa.front());

  result.push_back(d.front());
  result.push_back(dv.front());
  result.push_back(da.front());

  return result;
}

void Trajectory::pop_front() {
  s.pop_front();
  sv.pop_front();
  sa.pop_front();

  d.pop_front();
  dv.pop_front();
  da.pop_front();
}

void Trajectory::push_back(vector<double> dp) {
  s.push_back(dp[0]);
  sv.push_back(dp[1]);
  sa.push_back(dp[2]);

  d.push_back(dp[3]);
  dv.push_back(dp[4]);
  da.push_back(dp[5]);
}

void Trajectory::calculateStats() {
  avg_velocity_s = average(sv);
  avg_acceleration_s = average(sa);
  final_d = d.back();
  final_s = s.back();
  final_v = sv.back();
}

int Trajectory::size() {
  return s.size();
}

void Trajectory::printToStdout() const {
  cout << "-------------------------------"  << endl;
  cout << "| final_d = " << final_d << endl;
  cout << "| final_s = " << final_s << endl;
  cout << "| final_v = " << final_v << endl;
  cout << "|- s: " << endl;
  cout << "|   velocity  avg = " << avg_velocity_s << " max = " << max_velocity_s << " min = " << min_velocity_s << endl;
  cout << "|   acceleration  avg = " << avg_acceleration_s << " max = " << max_acceleration_s << endl;
  cout << "|   jerk  max = " << max_jerk_s << endl;
  cout << "|- d: " << endl;
  cout << "|   velocity  max = " << max_velocity_d << endl;
  cout << "|   acceleration  max = " << max_acceleration_d << endl;
  cout << "|   jerk  max = " << max_jerk_d << endl;
  cout << "-------------------------------"  << endl;
}

/********************************************************************************************
*
*     ImpossibleTrajectory
*
********************************************************************************************/

double ImpossibleTrajectory::cost(std::vector<std::vector<double>> sensor_fusion) const {
  return DBL_MAX;
}

void ImpossibleTrajectory::printToStdout() const {
  cout << "ImpossibleTrajectory" << endl;
}

