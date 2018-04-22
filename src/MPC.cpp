#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <float.h>

using CppAD::AD;

// Evaluate a polynomial.
AD<double> polyeval(Eigen::VectorXd coeffs, AD<double> x) {
  AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result = result + coeffs[i] * CppAD::pow(x, i);
  }
  return result;
};

void displaySolution(CPPAD_TESTVECTOR(double) solution, size_t N, bool ok){
  size_t x_start = 0;
  size_t y_start = N;
  size_t psi_start = N*2;
  size_t v_start = N*3;
  size_t cte_start = N*4;
  size_t epsi_start = N*5;
  size_t delta_start = N*6;
  size_t a_start = N*7-1;

  std::cout << "Solution" << std::endl;

  if(ok)
    std::cout << "solution found." << std::endl;
  else
    std::cout << "solution not found." << std::endl;

  size_t i =0;
  std::cout << "x:\t";
  for(i= x_start; i<x_start+N; ++i)
    std::cout << solution[i] << "\t";
  std::cout << std::endl;

  std::cout << "y:\t";
  for(i= y_start; i<y_start+N; ++i)
    std::cout << solution[i] << "\t";
  std::cout << std::endl;

  std::cout << "psi:\t";
  for(i= psi_start; i<psi_start+N; ++i)
    std::cout << solution[i] << "\t";
  std::cout << std::endl;

  std::cout << "v:\t";
  for(i= v_start; i<v_start+N; ++i)
    std::cout << solution[i] << "\t";
  std::cout << std::endl;

  std::cout << "cte:\t";
  for(i= cte_start; i<cte_start+N; ++i)
    std::cout << solution[i] << "\t";
  std::cout << std::endl;

  std::cout << "epsi:\t";
  for(i= epsi_start; i<epsi_start+N; ++i)
    std::cout << solution[i] << "\t";
  std::cout << std::endl;

  std::cout << "delta:\t";
  for(i= delta_start; i<delta_start+N-1; ++i)
    std::cout << solution[i] << "\t";
  std::cout << std::endl;

  std::cout << "a:\t";
  for(i= a_start; i<a_start+N-1; ++i)
    std::cout << solution[i] << "\t";
  std::cout << std::endl;
}

void displayVarUpperLowerBound(CPPAD_TESTVECTOR(double) upper, CPPAD_TESTVECTOR(double) lower, size_t N){
  size_t x_start = 0;
  size_t y_start = N;
  size_t psi_start = N*2;
  size_t v_start = N*3;
  size_t cte_start = N*4;
  size_t epsi_start = N*5;
  size_t delta_start = N*6;
  size_t a_start = N*7-1;

  std::cout << "Upper" << std::endl;

  size_t i =0;
  std::cout << "x:\t";
  for(i= x_start; i<x_start+N; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "y:\t";
  for(i= y_start; i<y_start+N; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "psi:\t";
  for(i= psi_start; i<psi_start+N; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "v:\t";
  for(i= v_start; i<v_start+N; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "cte:\t";
  for(i= cte_start; i<cte_start+N; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "epsi:\t";
  for(i= epsi_start; i<epsi_start+N; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "delta:\t";
  for(i= delta_start; i<delta_start+N-1; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "a:\t";
  for(i= a_start; i<a_start+N-1; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "Lower" << std::endl;

  std::cout << "x:\t";
  for(i= x_start; i<x_start+N; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;

  std::cout << "y:\t";
  for(i= y_start; i<y_start+N; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;

  std::cout << "psi:\t";
  for(i= psi_start; i<psi_start+N; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;

  std::cout << "v:\t";
  for(i= v_start; i<v_start+N; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;

  std::cout << "cte:\t";
  for(i= cte_start; i<cte_start+N; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;

  std::cout << "epsi:\t";
  for(i= epsi_start; i<epsi_start+N; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;

  std::cout << "delta:\t";
  for(i= delta_start; i<delta_start+N-1; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;

  std::cout << "a:\t";
  for(i= a_start; i<a_start+N-1; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;
}

void displayConstraintUpperLowerBound(CPPAD_TESTVECTOR(double) upper, CPPAD_TESTVECTOR(double) lower, size_t N){
  size_t x_start = 0;
  size_t y_start = N;
  size_t psi_start = N*2;
  size_t v_start = N*3;
  size_t cte_start = N*4;
  size_t epsi_start = N*5;
  size_t delta_start = N*6;
  size_t a_start = N*7-1;

  std::cout << "Upper" << std::endl;

  size_t i =0;
  std::cout << "x:\t";
  for(i= x_start; i<x_start+N; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "y:\t";
  for(i= y_start; i<y_start+N; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "psi:\t";
  for(i= psi_start; i<psi_start+N; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "v:\t";
  for(i= v_start; i<v_start+N; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "cte:\t";
  for(i= cte_start; i<cte_start+N; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "epsi:\t";
  for(i= epsi_start; i<epsi_start+N; ++i)
    std::cout << upper[i] << "\t";
  std::cout << std::endl;

  std::cout << "Lower" << std::endl;

  std::cout << "x:\t";
  for(i= x_start; i<x_start+N; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;

  std::cout << "y:\t";
  for(i= y_start; i<y_start+N; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;

  std::cout << "psi:\t";
  for(i= psi_start; i<psi_start+N; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;

  std::cout << "v:\t";
  for(i= v_start; i<v_start+N; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;

  std::cout << "cte:\t";
  for(i= cte_start; i<cte_start+N; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;

  std::cout << "epsi:\t";
  for(i= epsi_start; i<epsi_start+N; ++i)
    std::cout << lower[i] << "\t";
  std::cout << std::endl;
}


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  Eigen::VectorXd derivative_coeffs;
  size_t N;
  double dt;
  double Lf;
  double ref_v = 0;

  FG_eval(size_t N, double dt, double Lf, double ref_v, Eigen::VectorXd coeffs) {
    this->N = N;
    this->dt = dt;
    this->Lf = Lf;
    this->ref_v = ref_v;
    this->coeffs = coeffs;

    derivative_coeffs = Eigen::VectorXd::Zero(coeffs.size()-1);

    for(int i=1; i<coeffs.size(); ++i){
      derivative_coeffs[i-1] = i * coeffs[i];
    }

    /*
    std::cout << "FG_eval debug: " << N << " " << dt << " " << Lf << " " << ref_v << " " << std::endl;
    for(size_t i=0; i<coeffs.size(); ++i)
      std::cout << coeffs[i] << " " ;
    std::cout << std::endl;

    for(size_t i=0; i<derivative_coeffs.size(); ++i)
      std::cout << derivative_coeffs[i] << " " ;
    std::cout << std::endl;
    */
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.

    // The cost is stored is the first element of `fg`.
    // Any additions to the cost should be added to `fg[0]`.
    fg[0] = 0;

    // Cost function
    // TODO: Define the cost related the reference state and
    // any anything you think may be beneficial.
    const size_t x_start = 0;
    const size_t y_start = N;
    const size_t psi_start = N*2;
    const size_t v_start = N*3;
    const size_t cte_start = N*4;
    const size_t epsi_start = N*5;
    const size_t delta_start = N*6;
    const size_t a_start = N*7-1;

    /*
    std::cout << "debug mem layout: " << fg.size() << " " << x_start << " " << y_start << " " << psi_start << " " << v_start
              << " " << cte_start << " " << epsi_start << " " << delta_start << " " << a_start << std::endl;
              */

    // The part of the cost based on the reference state.
    for (size_t t = 0; t < N; t++) {
      fg[0] += CppAD::pow(vars[cte_start + t], 2);
      fg[0] += CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - ref_v, 2);
    }

    // Minimize the use of actuators.
    for (size_t t = 0; t < N - 1; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (size_t t = 0; t < N - 2; t++) {
      fg[0] += CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];


    for (size_t t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      AD<double> f0 = polyeval(coeffs, x0);
      AD<double> psides0 = CppAD::atan(polyeval(derivative_coeffs, x0));

      // Here's `x` to get you started.
      // The idea here is to constraint this value to be 0.
      //
      // Recall the equations for the model:
      // x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
      // y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
      // psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
      // v_[t] = v[t-1] + a[t-1] * dt
      // cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
      // epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);

      fg[1 + cte_start + t] =
          cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + t] =
          epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);

    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

void MPC::Init(size_t N, double dt, double Lf, double ref_v){
  this->N = N;
  this->dt = dt;
  this->ref_v = ref_v;
  this->Lf = Lf;
}


vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = 6 * N + 2 * (N-1);
  // TODO: Set the number of constraints
  size_t n_constraints = 6 * N;

  //std::cout << "debug n_vars: " << n_vars << " n_constraints: " << n_constraints << std::endl;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  //memory layout
  const size_t x_start = 0;
  const size_t y_start = N;
  const size_t psi_start = N*2;
  const size_t v_start = N*3;
  const size_t cte_start = N*4;
  const size_t epsi_start = N*5;
  const size_t delta_start = N*6;
  const size_t a_start = N*7-1;

  double x= state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.

  for (i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1e10;
    vars_upperbound[i] = 1e10;
  }

  for (i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  for (i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  //displayVarUpperLowerBound(vars_upperbound, vars_lowerbound, N);

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  //displayConstraintUpperLowerBound(constraints_upperbound, constraints_lowerbound, N);

  // object that computes objective and constraints
  FG_eval fg_eval(N, dt, Lf, ref_v, coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.

  displaySolution(solution.x, N, ok);

  vector<double> result;

  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  // use for display in the program
  for (i = 0; i < N-1; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }

  return result;
}
