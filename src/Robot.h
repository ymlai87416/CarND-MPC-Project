//
// Created by tom on 4/22/18.
//

#ifndef MPC_ROBOT_H
#define MPC_ROBOT_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "MPC.h"

using namespace std;

class Robot {
public:
  /*
   * Constructor
   */
  Robot();

  /*
   * Destructor
   */
  virtual ~Robot();

  /*
   * Init MPC
   */
  void Init(size_t num_step_forward, double step_interval, double reference_velocity);

  /*
   * calculate the steering angle and thottle
   */
  void calculateSteeringAngleAndThrottle(double x, double y, double psi, double speed,
                                         vector<double> ptsX, vector<double> ptsY,
                                         double& steeringAngle, double& throttle,
                                         vector<double>& trajectory_x, vector<double>& trajectory_y);


private:
  double polyeval(Eigen::VectorXd coeffs, double x);

  Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

  MPC mpc;

  double ref_v;   //may be I can remove it? how about that??

  size_t N;

  double dt;  //in  ms

  double Lf;
};


#endif //MPC_ROBOT_H
