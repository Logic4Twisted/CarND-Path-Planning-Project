#ifndef CAR_LOCATION_H_
#define CAR_LOCATION_H_

struct CarLocation {
  double car_x;
  double car_y;
  double car_s;
  double car_d;
  double car_yaw;
  double car_speed;

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
};

#endif /* CAR_LOCATION_H_ */