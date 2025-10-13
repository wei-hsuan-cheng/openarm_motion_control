#pragma once
#include "iqpsolver.hpp"

// Thin OSQP wrapper. Requires libosqp installed and discoverable by CMake.
//
// Equality constraints are handled by converting Aeq x = beq
// to A_ineq x <= b_ineq with two-sided inequalities (stacked).
// Alternatively, one can use KKT build; this version keeps it simple.
class OsqpSolver : public IQPSolver {
public:
  OsqpSolver();
  ~OsqpSolver() override;

  QPResult solve(const QPProblem& p) override;

  // Optional: set OSQP parameters
  void setRho(double rho);
  void setEpsAbs(double eps);
  void setEpsRel(double eps);
  void setMaxIter(int iters);
  void setVerbose(bool v);

private:
  struct Impl;
  Impl* impl_; // PIMPL to avoid leaking OSQP headers in this file
};
