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
#include "helper_functions.h"
#include <ctime>
#include <ratio>
#include <chrono>
#include <assert.h>
#include <unordered_map>
#include "map.h"

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

string minCostState(unordered_map<string, double> costs) {
  pair<string, double> min = *min_element(costs.begin(), costs.end(), pairCompare );
  return min.first;
}

HighwayMap highwayMap;

Trajectory previous;

double previous_car_speed, previous_car_acceleration;
double ref_vel;
string state;

deque<double> accels;
deque<double> jerks;

int target_lane;

bool can_change_lane = false;

chrono::high_resolution_clock::time_point timespamp;

void update_state(CarLocation egoLocation) {
  return;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  std::cout << std::setprecision(3) << std::fixed;

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
    highwayMap.addWaypoint(x, y, s, d_x, d_y);
  }

  target_lane = 1;
  previous_car_speed = 0.0;
  previous_car_acceleration = 0.0;
  ref_vel = 0.0;
  state = "CS";

  timespamp = chrono::high_resolution_clock::now();

  h.onMessage([](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
            currentLocation.car_lane = (int)(car_d/4.0);
            double diff_v = (car_speed - previous_car_speed)*1000/dt.count();
            double diff_a = (diff_v - previous_car_acceleration)*1000/dt.count();


            accels.push_back(diff_v);
            if (accels.size() >  3) accels.pop_front();
            currentLocation.car_acceleration = average(accels);

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

            bool in_lane = (fabs(4.0*target_lane + 2.0 - car_d) < 1.0);

            std::cout << "prev size = " << prev_size << ", dt = " << dt.count() << " ms " << endl ;
            std::cout << "position = (" << car_s <<  ", " << car_d << ") xy = (" << car_x << ", " << car_y << ")" << endl;
            std::cout << "speed = " << car_speed << " m/s = " << (car_speed*2.24) << " mi/h" << endl;
            std::cout << "ref speed = " << ref_vel << " m/s " << endl;
            std::cout << "acceleration = " << diff_v << " m/s2, (" << currentLocation.car_acceleration << ")" << endl;
            std::cout << "jerk = " << diff_a << endl;
            std::cout << "car angle = " << currentLocation.car_yaw << " degrees " << endl;
            std::cout << "look ahead = " << (end_path_s - car_s) << " m" << endl;
            std::cout << "state = " << state << endl;
            std::cout << "current lane = " << currentLocation.car_lane << ", target lane = " << target_lane << (in_lane?" IN LANE ":" CHANGING LANE ") << endl; 
            
            
            previous_car_speed = car_speed;
            previous_car_acceleration = diff_v;

            if (car_speed*2.24 >= 50.0) {
              cout << "!! Speed !!" << endl;
            }
            if (fabs(diff_v) >= 10.0) {
              cout << "!! Acceleration !!" << endl;
            }
            if (fabs(diff_a) >= 50) {
              cout << "!! Jerk !!" << endl;
            }

            vector<double> closest_in_this_lane = closest_in_front(sensor_fusion, (int)(car_d/4.0), car_s, 50.0);
            if (closest_in_this_lane.size() == 0) {
              cout << "No car in next 50 meters." << endl;
            }
            else {
              cout << "Distance to next car in lane = " << (closest_in_this_lane[1] - car_s) << " m" << endl;
            }

            Trajectory selectedTrajectory;
            if (in_lane) {
              unordered_map<string, Trajectory> map = Trajectory::generate(previous, currentLocation, highwayMap, 4.0);
            
              unordered_map<string, double> costs;
              for (auto const& x : map) {
                cout << x.first << "(";
                costs[x.first] = x.second.cost(sensor_fusion);
                cout << ") :: " << costs[x.first] << endl;
              }
              state = minCostState(costs);
              cout << "Min cost State = " << state << endl;

              selectedTrajectory = map[state];
            }
            else {
              selectedTrajectory = Trajectory::reuseTrajectory(previous, currentLocation, highwayMap);
            }
            

            selectedTrajectory.printToStdout();

            target_lane = selectedTrajectory.getTargetLane();

            vector<double> next_x_vals;
            vector<double> next_y_vals;
            for (int i = 0; i < min(selectedTrajectory.size(), 50); i++) {
              vector<double> xy = highwayMap.getXY(selectedTrajectory.s[i], selectedTrajectory.d[i]);
              next_x_vals.push_back(xy[0]);
              next_y_vals.push_back(xy[1]);
            }

            cout << "Sent " << next_x_vals.size() << " as Trajectory " << endl;


            json msgJson;
          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

            previous = selectedTrajectory;

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
