#include "helper_functions.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <math.h>
#include "spline.h"

using namespace std;
using namespace Eigen;

double distance(double x1, double y1, double x2, double y2) {
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double miph_to_kmph(double miph) {
	return miph*1.60934;
}

double miph_to_mps(double miph) {
	return miph_to_kmph(miph)/3.6;
}

double logistic(double x) {
	return 2.0/(1.0 + exp(0.0 - x)) - 1.0;
}

double average(deque<double> dq) {
	double sum = 0.0;
	for (int i = 0; i < dq.size(); i++) {
		sum += dq[i];
	}
	return sum/dq.size();
}

vector<double> closest_in_front(vector<vector<double>> sensor_fusion, int lane, double s, double range) {
  vector<double> result;
  double curr_min = range;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    double d = sensor_fusion[i][6];
    if (d > 4.0*lane && d < 4.0*lane+4.0) {
      double vx = sensor_fusion[i][3];
      double vy = sensor_fusion[i][4];
      double v = sqrt(vx*vx + vy*vy);
      double check_car_s = sensor_fusion[i][5];
      double dist = fmod(check_car_s - s + TRACK_LEN, TRACK_LEN);
      if (dist > 0 && dist < curr_min) {
        curr_min = dist;
        result.clear();
        result.push_back(i);
        result.push_back(check_car_s);
        result.push_back(v);
      }
    }
  }
  return result;
}

// Calculates Jerk Minimizing Trajectory for start, end and T.
vector<double> jmt(vector<double> start, vector<double> final, double T) {
  assert(T > 0.0);
  assert(start.size() == 3);
  assert(final.size() == 3);

  double a0 = start[0];
  double a1 = start[1];
  double a2 = start[2]/2.0;

  double c0 = a0 + a1*T + a2*pow(T, 2.0);
  double c1 = a1 + 2.0*a2*T;
  double c2 = 2.0*a2;

  MatrixXd B = MatrixXd(3,1);
  B << final[0] - c0, final[1] - c1, final[2] - c2;

  MatrixXd A = MatrixXd(3,3);
  A << pow(T, 3.0), pow(T, 4.0), pow(T, 5.0),
       3.0*pow(T, 2.0), 4.0*pow(T, 3.0), 5.0*pow(T, 4.0),
       6.0*T, 12.0*pow(T, 2.0), 20.0*pow(T, 3.0);

  MatrixXd X = A.inverse() * B;

  vector<double> result;
  result.push_back(a0);
  result.push_back(a1);
  result.push_back(a2);
  result.push_back(X(0,0));
  result.push_back(X(1,0));
  result.push_back(X(2,0));
  return result;
}

double eval(vector<double> coeff, double x) {
  assert(coeff.size() > 0);
  double result = coeff[0];
  for (int i = 1; i < coeff.size(); i++) {
    result += coeff[i]*pow(x, i);
  }
  return result;
}

vector<double> dif(vector<double> coeff) {
  assert(coeff.size() > 0);
  vector<double> result;
  for (int i = 1; i < coeff.size(); i++) {
    result.push_back(coeff[i]*i);
  }
  return result;
}