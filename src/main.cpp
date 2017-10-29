#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <map>
#include <string>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include "json.hpp"
#include "cost_functions.h"
#include "car_location.h"
#include <ctime>
#include <ratio>
#include <chrono>
#include <assert.h>

using namespace std;
using namespace Eigen;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

bool pairCompare( pair<string,double> i, pair<string, double> j) {
  return i.second < j.second;
}

string minCostState(map<string, double> costs) {
  pair<string, double> min = *min_element(costs.begin(), costs.end(), pairCompare );
  return min.first;
}

vector<double> map_waypoints_x;
vector<double> map_waypoints_y;
vector<double> map_waypoints_s;
vector<double> map_waypoints_dx;
vector<double> map_waypoints_dy;

double previous_car_speed;
double ref_vel;
string state;

chrono::high_resolution_clock::time_point timespamp;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }

  int lane = 1;
  previous_car_speed = 0.0;
  ref_vel = 0.0;
  state = "CS";

  timespamp = chrono::high_resolution_clock::now();

  h.onMessage([&lane](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];
            car_speed /= 2.24;

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

            chrono::high_resolution_clock::time_point t2 = chrono::high_resolution_clock::now();
            chrono::duration<double, std::milli> dt = t2 - timespamp;
            timespamp = t2;


            std::cout << "***********************************************************" << endl;

            CarLocation currentLocation;
            currentLocation.car_x = car_x;
            currentLocation.car_y = car_y;
            currentLocation.car_s = car_s;
            currentLocation.car_d = car_d;
            currentLocation.car_yaw = car_yaw;
            currentLocation.car_speed = car_speed;
            currentLocation.car_lane = lane;
            currentLocation.car_acceleration = (car_speed - previous_car_speed)*1000/dt.count();

            for (int i = 0; i < previous_path_x.size(); i++) {
              currentLocation.previous_path_x.push_back(previous_path_x[i]);
            }
            for (int i = 0; i < previous_path_y.size(); i++) {
              currentLocation.previous_path_y.push_back(previous_path_y[i]);
            }
            currentLocation.end_path_s = end_path_s;
            currentLocation.end_path_d = end_path_d;


          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            int prev_size = previous_path_x.size();

            double max_accel = 5.0; // m/s^2

            std::cout << "prev size = " << prev_size << endl;
            std::cout << "car position = (" << car_s <<  ", " << car_d << ") xy = (" << car_x << ", " << car_y << ")" << endl;
            std::cout << "car speed = " << car_speed << " m/s = " << (car_speed*2.24) << " mi/h" << endl;
            std::cout << "ref speed = " << ref_vel << " m/s " << endl;
            std::cout << "car accel = " << currentLocation.car_acceleration << " m/s2" << endl;
            std::cout << "car angle = " << currentLocation.car_yaw << " degrees " << endl;
            std::cout << "look ahead = " << (end_path_s - car_s) << " m" << endl;
            std::cout << "state = " << state << endl;

            std::cout << "lane = " << lane << endl; 
            bool in_lane = (fabs(4.0*lane + 2.0 - car_d) < 1.0);
            previous_car_speed = car_speed;

            map<string, Target> targets;
            map<string, double> costs;
            map<string, Trajectory> map;
            
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            if (state == "CL-L" || state == "CL-R") {
              if (in_lane) {
                state = "CS";
              }
              for (int i = 0; i < previous_path_x.size(); i++) {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
              }

            }
            else {
              Target target1;
              target1.v = ref_vel;
              target1.lane = lane;
              target1.time = 0.0;
              target1.s = 0.0;
              target1.d = 0.0;

              cout << "CS" << endl;
              Trajectory trajectory0 = Trajectory::create_trajectory(currentLocation, target1, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              map["CS"] = trajectory0;
              targets["CS"] = target1;

              if (lane > 0 && prev_size > 1 && car_speed > 5.0 && car_speed + currentLocation.car_acceleration*3.0 > 0) {
                cout << "CL-L" << endl;
                Target target2;
                target2.v = min(49.0, car_speed + currentLocation.car_acceleration*3.0);
                target2.lane = lane - 1;
                target2.s = car_s + currentLocation.car_speed*3.0 + currentLocation.car_acceleration*pow(3.0, 2)/2.0;
                target2.a = currentLocation.car_acceleration;
                Trajectory trajectory2 = Trajectory::create_CL_trajectory(currentLocation, target2, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                map["CL-L"] = trajectory2;
                targets["CL-L"] = target2;
              }


              if (lane < 2 && prev_size > 1 && car_speed > 5.0 && car_speed + currentLocation.car_acceleration*3.0 > 0) {
                cout << "CL-R" << endl;
                Target target3;
                target3.v = min(49.0, car_speed + currentLocation.car_acceleration*3.0);
                target3.lane = lane + 1;
                target3.s = car_s + currentLocation.car_speed*3.0 + currentLocation.car_acceleration*pow(3.0, 2.0)/2.0;
                target3.a = currentLocation.car_acceleration;
                Trajectory trajectory3 = Trajectory::create_CL_trajectory(currentLocation, target3, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                map["CL-R"] = trajectory3;
                targets["CL-R"] = target3;
              }

              cout << "KL+" << endl;
              Target target4;
              target4.v = ref_vel + 0.1;
              target4.lane = lane;
              Trajectory trajectory4 = Trajectory::create_trajectory(currentLocation, target4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              map["KL+"] = trajectory4;
              targets["KL+"] = target4;

              cout << "KL++" << endl;
              Target target5;
              target5.v = ref_vel + 0.1*2;
              target5.lane = lane;
              Trajectory trajectory5 = Trajectory::create_trajectory(currentLocation, target5, map_waypoints_s, map_waypoints_x, map_waypoints_y);
              map["KL++"] = trajectory5;
              targets["KL++"] = target5;

              if (ref_vel > 0.2) {
                cout << "KL-" << endl;
                Target target4;
                target4.v = ref_vel - 0.1;
                target4.lane = lane;
                Trajectory trajectory4 = Trajectory::create_trajectory(currentLocation, target4, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                map["KL-"] = trajectory4;
                targets["KL-"] = target4;
              }

              if(ref_vel > 0.5) {
                cout << "KL--" << endl;
                Target target5;
                target5.v = ref_vel - 0.1*2;
                target5.lane = lane;
                Trajectory trajectory5 = Trajectory::create_trajectory(currentLocation, target5, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                map["KL--"] = trajectory5;
                targets["KL--"] = target5;
              }

              for (auto const& x : map) {
                cout << x.first << "(";
                costs[x.first] = x.second.cost(sensor_fusion);
                cout << ") :: " << costs[x.first] << endl;
              }

              state = minCostState(costs);
              cout << "Min cost State = " << state << endl;
              Trajectory trajectory = map[state];
              ref_vel = trajectory.final_v;
              lane = targets[state].lane;

              for (int i = 0; i < trajectory.x.size(); i++) {
                next_x_vals.push_back(trajectory.x[i]);
                next_y_vals.push_back(trajectory.y[i]);
              }
            }

            json msgJson;
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
