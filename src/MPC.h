#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);

  //init
  void Init(size_t N, double dt, double Lf, double ref_v);

private:
  size_t N = 10;
  double dt = 0.1;
  double Lf = 2.67;
  double ref_v = 70;
};

#endif /* MPC_H */
