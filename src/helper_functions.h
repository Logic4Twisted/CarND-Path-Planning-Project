#ifndef CAR_LOCATION_H_
#define CAR_LOCATION_H_

#include <math.h>
#include <vector>
#include <deque>

struct CarLocation {
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;
  int car_lane;
  double car_acceleration;

  std::vector<double> previous_path_x;
  std::vector<double> previous_path_y;

  double end_path_s;
  double end_path_d;
};

struct Target {
  double s;
  double d;
  double time;
  double v;
  double lane;
  double a;
};

const double TRACK_LEN = 6945.554;

double miph_to_kmph(double miph);
double miph_to_mps(double miph);

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
inline double sgn(double x) { if (x > 0.0) return 1.0; return 0.0;}

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y, const std::vector<double> &maps_dx, const std::vector<double> &maps_dy);

std::vector<double> closest_in_front(std::vector<std::vector<double>> sensor_fusion, int lane, double s, double range);

std::vector<double> jmt(std::vector<double> start, std::vector<double> final, double T);

double eval(std::vector<double> coeff, double x);

std::vector<double> dif(std::vector<double> coeff);

double average(std::deque<double> dq);

double logistic(double x);

#endif /* CAR_LOCATION_H_ */