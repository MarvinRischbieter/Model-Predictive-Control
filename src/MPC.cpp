#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

# define DEG25 0.436332  // 25 deg in rad

// Weights for the loss for fast race. Uncomment for fast race (and comment lines 19-26). May be unstable.
//# define VEL_REF 41  // Desired speed in m/s
//# define CTE 110
//# define EPSI 1.5
//# define VEL 3.0
//# define DELTA 5e5
//# define ACC 20
//# define D_DELTA 2.0
//# define D_ACC 0.02

// Weights for the loss for slow and safe race. Uncomment for slow race (and comment lines 9-16). 
# define VEL_REF 20  // Desired speed in m/s
# define CTE 35
# define EPSI 15.0
# define VEL 12.0
# define DELTA 1e5
# define ACC 20
# define D_DELTA 0.2
# define D_ACC 0.01

using CppAD::AD;

// Set the timestep length and duration
size_t N = 15;
double dt = 0.1;

const double vel_ref = VEL_REF;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) {
    this->coeffs = coeffs;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    fg[0] = 0;
    // Minimize the cte and epsi
    unsigned int i;
    for (i = 0; i < N; i++) {
      fg[0] += CTE*CppAD::pow(vars[i + cte_start], 2);  // Desired cte - 0
      fg[0] += EPSI*CppAD::pow(vars[i + epsi_start], 2);  // Desired epsi - 0
      fg[0] += VEL*CppAD::pow(vars[i + v_start] - vel_ref, 2);
    }

   // Minimize the use of actuators
    for (i = 0; i < N - 1; i++) {
      fg[0] += DELTA*CppAD::pow(vars[i + delta_start], 2);
      fg[0] += ACC*CppAD::pow(vars[i + a_start], 2);
    }

    // Minimize the value gap between sequential actuations
    for (i = 0; i < N - 2; i++) {
      fg[0] += D_DELTA*CppAD::pow(vars[i + 1 + delta_start] - vars[i + delta_start], 2);
      fg[0] += D_ACC*CppAD::pow(vars[i + 1 + a_start] - vars[i + a_start], 2);
    }

    // Initial constraints
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (i = 1; i < N; i++) {

      // The state at time t+1
      AD<double> x1 = vars[x_start + i];
      AD<double> y1 = vars[y_start + i];
      AD<double> psi1 = vars[psi_start + i];
      AD<double> v1 = vars[v_start + i];
      AD<double> cte1 = vars[cte_start + i];
      AD<double> epsi1 = vars[epsi_start + i];
      
      // The state at time t
      AD<double> x0 = vars[x_start + i - 1];
      AD<double> y0 = vars[y_start + i - 1];
      AD<double> psi0 = vars[psi_start + i - 1];
      AD<double> v0 = vars[v_start + i - 1];
      AD<double> cte0 = vars[cte_start + i - 1];
      AD<double> epsi0 = vars[epsi_start + i - 1];
      
      // The actuation at time t
      AD<double> delta0 = vars[delta_start + i - 1];
      AD<double> a0 = vars[a_start + i - 1];

      AD<double> f0 = 0.0;
      for (int j = 0; j < coeffs.size(); j++) {
        f0 += coeffs[j] * CppAD::pow(x0, j);  // f(x0)
      }
      AD<double> psides0 = 0.0;
      for (int j = 1; j < coeffs.size(); j++) {
        psides0 += j*coeffs[j] * CppAD::pow(x0, j-1); // f'(x0)
      }
      psides0 = CppAD::atan(psides0);

      //Setup the rest of the model constraints
      fg[1 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + i] = psi1 - (psi0 + (v0/Lf) * delta0 * dt);
      fg[1 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[1 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  unsigned int i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = 6 * N + 2 * (N - 1);
  // Set the number of constraints
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }
  // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set lower and upper limits for variables.
  for (i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -numeric_limits<float>::max();
    vars_upperbound[i] = numeric_limits<float>::max();
  }

  // Don't forget to include Lf in delta values if dividing by Lf in main while making the delta
  // or steering angles to [-1, 1] and a values are in the range [-1, 1]
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  for (i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -DEG25;
    vars_upperbound[i] = DEG25;
  }

  // Acceleration/decceleration upper and lower limits.
  for (i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

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

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

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

  //Return the first actuator values along with the predicted position of vehicle. The predicted
  //positions are used to plot the trajectory in simulator.

  // Store the MPC predicted trajectory
  this->mpc_x_vals = {};
  this->mpc_y_vals = {};
  for (i = 0; i < N; i++) {
      this->mpc_x_vals.push_back(solution.x[x_start + i]);
      this->mpc_y_vals.push_back(solution.x[y_start + i]);
  }
  std::vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (i = 0; i < N - 1; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }
  return result;
}
