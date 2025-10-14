#include "mmc_qp_utils/osqp_solver.hpp"
#include <osqp.h>
#include <vector>
#include <cassert>
#include <cmath>
#include <iostream>

struct OsqpSolver::Impl {
  OSQPSettings* settings{nullptr};
  OSQPSolver*   solver{nullptr};
  bool verbose{false};

  Impl() {
    settings = OSQPSettings_new();
    osqp_set_default_settings(settings);
    settings->alpha    = 1.6;
    settings->eps_abs  = 1e-4;
    settings->eps_rel  = 1e-4;
    settings->max_iter = 4000;
    settings->verbose  = 0;
  }
  ~Impl() {
    if (solver) osqp_cleanup(solver);
    if (settings) OSQPSettings_free(settings);
  }
};

OsqpSolver::OsqpSolver() : impl_(new Impl) {}
OsqpSolver::~OsqpSolver() { delete impl_; }

void OsqpSolver::setRho(double rho){ impl_->settings->rho = (OSQPFloat)rho; }
void OsqpSolver::setEpsAbs(double eps){ impl_->settings->eps_abs = (OSQPFloat)eps; }
void OsqpSolver::setEpsRel(double eps){ impl_->settings->eps_rel = (OSQPFloat)eps; }
void OsqpSolver::setMaxIter(int iters){ impl_->settings->max_iter = (OSQPInt)iters; }
void OsqpSolver::setVerbose(bool v){ impl_->settings->verbose = v ? 1 : 0; impl_->verbose = v; }

static void denseToCscUpper(const Eigen::MatrixXd& M,
                            std::vector<OSQPInt>& Ap, std::vector<OSQPInt>& Ai, std::vector<OSQPFloat>& Ax)
{
  const OSQPInt rows = (OSQPInt)M.rows();
  const OSQPInt cols = (OSQPInt)M.cols();
  Ap.resize(cols+1);
  std::vector<OSQPFloat> vals;
  std::vector<OSQPInt>   idx;
  vals.reserve((size_t)rows*(size_t)cols);
  idx.reserve((size_t)rows*(size_t)cols);
  OSQPInt count = 0;
  for (OSQPInt j=0;j<cols;++j) {
    Ap[(size_t)j] = count;
    for (OSQPInt i=0;i<rows;++i) {
      if (i>j) continue; // upper triangular
      const double v = M((int)i,(int)j);
      if (std::abs(v) > 0.0) {
        vals.push_back((OSQPFloat)v);
        idx.push_back(i);
        ++count;
      }
    }
  }
  Ap[(size_t)cols] = count;
  Ai = std::move(idx);
  Ax = std::move(vals);
}

static void denseToCsc(const Eigen::MatrixXd& M,
                       std::vector<OSQPInt>& Ap, std::vector<OSQPInt>& Ai, std::vector<OSQPFloat>& Ax)
{
  const OSQPInt rows = (OSQPInt)M.rows();
  const OSQPInt cols = (OSQPInt)M.cols();
  Ap.resize(cols+1);
  std::vector<OSQPFloat> vals;
  std::vector<OSQPInt>   idx;
  vals.reserve((size_t)rows*(size_t)cols);
  idx.reserve((size_t)rows*(size_t)cols);
  OSQPInt count = 0;
  for (OSQPInt j=0;j<cols;++j) {
    Ap[(size_t)j] = count;
    for (OSQPInt i=0;i<rows;++i) {
      const double v = M((int)i,(int)j);
      if (std::abs(v) > 0.0) {
        vals.push_back((OSQPFloat)v);
        idx.push_back(i);
        ++count;
      }
    }
  }
  Ap[(size_t)cols] = count;
  Ai = std::move(idx);
  Ax = std::move(vals);
}

// Stack inequalities: [Aineq;  Aeq; -Aeq] x <= [bineq; beq; -beq]
static void stackIneqWithEq(const QPProblem& p,
                            Eigen::MatrixXd& Aall, Eigen::VectorXd& ball)
{
  const int ni = (int)p.Aineq.rows();
  const int ne = (int)p.Aeq.rows();
  const int n  = (int)p.Q.rows();
  Aall.resize(ni + 2*ne, n);
  ball.resize(ni + 2*ne);

  if (ni>0) {
    Aall.topRows(ni) = p.Aineq;
    ball.head(ni)    = p.bineq;
  }
  if (ne>0) {
    Aall.middleRows(ni, ne) = p.Aeq;
    ball.segment(ni, ne)    = p.beq;
    Aall.bottomRows(ne)     = -p.Aeq;
    ball.tail(ne)           = -p.beq;
  }
}

QPResult OsqpSolver::solve(const QPProblem& p)
{
  QPResult r;
  const int n = (int)p.Q.rows();
  assert(p.Q.cols()==n);
  assert(p.c.size()==n);
  assert(p.lb.size()==n);
  assert(p.ub.size()==n);

  // Build A by stacking inequalities and bounds as OSQP "A x in [l,u]"
  Eigen::MatrixXd AineqEq;
  Eigen::VectorXd bineqEq;
  stackIneqWithEq(p, AineqEq, bineqEq);

  const int mI = (int)AineqEq.rows();
  Eigen::MatrixXd Aall(mI + n, n);
  Aall.setZero();
  if (mI>0) Aall.topRows(mI) = AineqEq;
  Aall.bottomRows(n) = Eigen::MatrixXd::Identity(n,n);

  Eigen::VectorXd lower = Eigen::VectorXd::Constant(mI + n, -OSQP_INFTY);
  Eigen::VectorXd upper(mI + n);
  if (mI>0) upper.head(mI) = bineqEq; // AineqEq x <= bineqEq
  upper.tail(n) = p.ub;               // I x in [lb, ub]
  lower.tail(n) = p.lb;

  // Convert to CSC
  std::vector<OSQPInt> Pp, Pi, Ap, Ai;
  std::vector<OSQPFloat> Px, Ax;
  const Eigen::MatrixXd Qsym = 0.5 * (p.Q + p.Q.transpose());
  denseToCscUpper(Qsym, Pp, Pi, Px); // OSQP expects upper-triangular part
  denseToCsc(Aall, Ap, Ai, Ax);

  // Prepare vectors q, l, u
  std::vector<OSQPFloat> q((size_t)n);
  for (int i=0;i<n;++i) q[(size_t)i] = (OSQPFloat)p.c(i);
  const int m = (int)Aall.rows();
  std::vector<OSQPFloat> l((size_t)m), u((size_t)m);
  for (int i=0;i<m;++i) { l[(size_t)i] = (OSQPFloat)lower(i); u[(size_t)i] = (OSQPFloat)upper(i); }

  // Wrap CSC matrices (no copy of arrays)
  OSQPCscMatrix* P = OSQPCscMatrix_new((OSQPInt)n, (OSQPInt)n, (OSQPInt)Px.size(),
                                       Px.data(), Pi.data(), Pp.data());
  OSQPCscMatrix* A = OSQPCscMatrix_new((OSQPInt)m, (OSQPInt)n, (OSQPInt)Ax.size(),
                                       Ax.data(), Ai.data(), Ap.data());

  // Re-setup solver each call (simple path)
  if (impl_->solver) {
    osqp_cleanup(impl_->solver);
    impl_->solver = nullptr;
  }
  OSQPSolver* solver = nullptr;
  OSQPInt status = osqp_setup(&solver,
                              P, q.data(),
                              A, l.data(), u.data(),
                              (OSQPInt)m, (OSQPInt)n,
                              impl_->settings);
  if (status != 0 || !solver) {
    if (impl_->verbose) std::cerr << "[OSQP] setup failed: " << status << "\n";
    if (P) OSQPCscMatrix_free(P);
    if (A) OSQPCscMatrix_free(A);
    r.ok = false; return r;
  }
  impl_->solver = solver;

  // Solve
  osqp_solve(impl_->solver);

  // Extract solution
  const OSQPSolution* sol = impl_->solver->solution;
  const OSQPInfo* info = impl_->solver->info;
  if (impl_->verbose && info) {
    std::cerr << "[OSQP] status=" << info->status_val
              << " iters=" << info->iter
              << " obj=" << info->obj_val
              << " prim_res=" << info->prim_res
              << " dual_res=" << info->dual_res
              << "\n";
  }
  r.ok = (info && (info->status_val == OSQP_SOLVED || info->status_val == OSQP_SOLVED_INACCURATE));
  r.iters = info ? (int)info->iter : 0;
  r.obj = info ? (double)info->obj_val : 0.0;
  r.x.resize(n);
  if (sol && sol->x) for (int i=0;i<n;++i) r.x(i) = (double)sol->x[i];

  // Free wrappers (they only free wrapper unless owned=1)
  if (P) OSQPCscMatrix_free(P);
  if (A) OSQPCscMatrix_free(A);
  return r;
}
