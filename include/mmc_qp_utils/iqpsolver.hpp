#pragma once
#include <Eigen/Dense>

struct QPResult {
  Eigen::VectorXd x;   // solution vector
  bool ok{false};      // solved?
  int iters{0};        // iterations (if available)
  double obj{0.0};     // objective value (if available)
};

struct QPProblem {
  // Minimize: 0.5 x^T Q x + c^T x
  // Subject to:
  //   Aeq x = beq
  //   Aineq x <= bineq
  //   lb <= x <= ub
  Eigen::MatrixXd Q;         // SPD / PSD (dim: n x n)
  Eigen::VectorXd c;         // n
  Eigen::MatrixXd Aeq;       // me x n  (me can be 0)
  Eigen::VectorXd beq;       // me
  Eigen::MatrixXd Aineq;     // mi x n  (mi can be 0)
  Eigen::VectorXd bineq;     // mi
  Eigen::VectorXd lb;        // n (can be -inf)
  Eigen::VectorXd ub;        // n (can be +inf)
};

class IQPSolver {
public:
  virtual ~IQPSolver() = default;
  virtual QPResult solve(const QPProblem& p) = 0;
};
