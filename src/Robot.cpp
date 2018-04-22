//
// Created by tom on 4/22/18.
//

#include "Robot.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

Robot::Robot(){

}

Robot::~Robot(){

}

void Robot::Init(size_t num_step_forward, double step_interval, double reference_velocity){
  this->N = num_step_forward;
  this->dt = step_interval;
  this->ref_v = reference_velocity;
  this->Lf = 2.67;

  mpc.Init(this->N, this->dt, this->Lf, this->ref_v);
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd Robot::polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

double Robot::polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result = result + coeffs[i] * pow(x, i);
  }
  return result;
}

void Robot::calculateSteeringAngleAndThrottle(double x, double y, double psi, double speed,
                                              vector<double> ptsX, vector<double> ptsY,
                                              double& steeringAngle, double& throttle,
                                              vector<double>& trajectory_x, vector<double>& trajectory_y){

  Eigen::VectorXd vector_ptsx = Eigen::VectorXd::Map(ptsX.data(), ptsX.size());
  Eigen::VectorXd vector_ptsy = Eigen::VectorXd::Map(ptsY.data(), ptsY.size());

  Eigen::VectorXd path = polyfit(vector_ptsx, vector_ptsy, 3);

  Eigen::VectorXd path_derivative = Eigen::VectorXd(2);
  path_derivative[0] = path[1];
  path_derivative[1] = 2*path[2];

  double cte = 0 - polyeval(path, x);
  double epsi = 0 - atan(polyeval(path_derivative, x));

  Eigen::VectorXd state = Eigen::VectorXd(6);
  state << 0, 0, 0, speed, cte, epsi;

  vector<double> result = mpc.Solve(state, path);

  steeringAngle = result[0];
  throttle = result[1];

  trajectory_x.clear();
  for(size_t i=0; i<N-1; ++i)
    trajectory_x.push_back(result[i]+2);

  trajectory_y.clear();
  for(size_t i=0; i<N-1; ++i)
    trajectory_y.push_back(result[i]+N+2);
}
