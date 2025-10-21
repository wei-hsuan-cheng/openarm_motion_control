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

  // Persistent problem data
  bool initialized{false};
  OSQPInt nvar{0}, me{0}, mi{0}, nb{0}, m{0};
  OSQPInt nnzP{0}, nnzA{0};
  std::vector<OSQPInt> Pp, Pi, Ap, Ai; // structure
  std::vector<OSQPFloat> Px, Ax;       // values
  std::vector<OSQPFloat> q, l, u;      // vectors

  Impl() {
    settings = OSQPSettings_new();
    osqp_set_default_settings(settings);
    settings->alpha    = 1.6;
    settings->eps_abs  = 1e-4;
    settings->eps_rel  = 1e-4;
    settings->max_iter = 4000;
    settings->warm_starting = 1;
    settings->polishing = 0;
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

static inline void build_P_diag_pattern(OSQPInt nvar,
                                        std::vector<OSQPInt>& Pp,
                                        std::vector<OSQPInt>& Pi)
{
  Pp.resize((size_t)nvar + 1);
  Pi.resize((size_t)nvar);
  for (OSQPInt j=0;j<nvar;++j) {
    Pp[(size_t)j] = j;
    Pi[(size_t)j] = j; // diagonal only
  }
  Pp[(size_t)nvar] = nvar;
}

// Build fixed A pattern: [Eq( me x nvar ); Damper( mi x nvar ); I( nb x nvar )]
static inline void build_A_pattern(OSQPInt n, OSQPInt nvar, OSQPInt me, OSQPInt mi, OSQPInt nb,
                                   std::vector<OSQPInt>& Ap,
                                   std::vector<OSQPInt>& Ai)
{
  Ap.resize((size_t)nvar + 1);
  std::vector<OSQPInt> rows;
  rows.reserve((size_t)(me*nvar + 2*n + nvar));
  OSQPInt nnz = 0;
  for (OSQPInt j=0;j<nvar;++j) {
    Ap[(size_t)j] = nnz;
    // Eq rows 0..me-1
    for (OSQPInt r=0;r<me;++r) { rows.push_back(r); ++nnz; }
    // Damper rows (two per joint column) located after eq block
    if (j < n) {
      rows.push_back(me + 2*j + 0); ++nnz; // lower: -qdot_j <= rhs
      rows.push_back(me + 2*j + 1); ++nnz; // upper: +qdot_j <= rhs
    }
    // Identity bounds row
    rows.push_back(me + mi + j); ++nnz;
  }
  Ap[(size_t)nvar] = nnz;
  Ai = std::move(rows);
}

// Fill A values into Ax following the fixed pattern above
static inline void fill_A_values(const Eigen::MatrixXd& Aeq,
                                 OSQPInt n, OSQPInt nvar, OSQPInt me, OSQPInt mi, OSQPInt nb,
                                 std::vector<OSQPFloat>& Ax)
{
  const size_t nnzA = (size_t)(me*nvar + 2*n + nvar);
  Ax.resize(nnzA);
  size_t k = 0;
  for (OSQPInt j=0;j<nvar;++j) {
    // Eq block values for column j
    for (OSQPInt r=0;r<me;++r) {
      Ax[k++] = (OSQPFloat)Aeq((int)r,(int)j);
    }
    // Damper values
    if (j < n) {
      Ax[k++] = (OSQPFloat)(-1.0); // lower
      Ax[k++] = (OSQPFloat)(+1.0); // upper
    }
    // Identity bounds
    Ax[k++] = (OSQPFloat)(1.0);
  }
}

// Prepare l,u stacked vectors for [Eq; Inequality; Bounds]
static inline void fill_lu(const Eigen::VectorXd& beq,
                           const Eigen::VectorXd& bineq,
                           const Eigen::VectorXd& lb,
                           const Eigen::VectorXd& ub,
                           OSQPInt me, OSQPInt mi, OSQPInt nb,
                           std::vector<OSQPFloat>& l,
                           std::vector<OSQPFloat>& u)
{
  l.resize((size_t)(me+mi+nb));
  u.resize((size_t)(me+mi+nb));
  // Eq: l=u=beq
  for (OSQPInt i=0;i<me;++i) { l[i]=(OSQPFloat)beq((int)i); u[i]=(OSQPFloat)beq((int)i); }
  // Inequality: l=-inf, u=bineq
  for (OSQPInt i=0;i<mi;++i) { l[me+i]=-(OSQPFloat)OSQP_INFTY; u[me+i]=(OSQPFloat)bineq((int)i); }
  // Bounds: l=lb, u=ub
  for (OSQPInt i=0;i<nb;++i) { l[me+mi+i]=(OSQPFloat)lb((int)i); u[me+mi+i]=(OSQPFloat)ub((int)i); }
}

// Diagonal Px from Q diag (upper triangular)
static inline void fill_P_diag(const Eigen::MatrixXd& Q,
                               std::vector<OSQPFloat>& Px)
{
  const OSQPInt nvar = (OSQPInt)Q.rows();
  Px.resize((size_t)nvar);
  for (OSQPInt j=0;j<nvar;++j) Px[(size_t)j] = (OSQPFloat)Q((int)j,(int)j);
}

QPResult OsqpSolver::solve(const QPProblem& p)
{
  QPResult r;
  const OSQPInt nvar = (OSQPInt)p.Q.rows();
  const OSQPInt me = (OSQPInt)p.Aeq.rows();
  const OSQPInt mi = (OSQPInt)p.Aineq.rows();
  const OSQPInt nb = nvar; // identity block for variable bounds

  // First-time setup or dimension change
  bool need_setup = (!impl_->initialized || impl_->nvar!=nvar || impl_->me!=me || impl_->mi!=mi);

  if (need_setup) {
    if (impl_->solver) { osqp_cleanup(impl_->solver); impl_->solver=nullptr; }

    impl_->nvar = nvar; impl_->me = me; impl_->mi = mi; impl_->nb = nb; impl_->m = me + mi + nb;

    // P pattern (diagonal)
    build_P_diag_pattern(nvar, impl_->Pp, impl_->Pi);
    impl_->nnzP = nvar;
    fill_P_diag(p.Q, impl_->Px);

    // A pattern
    build_A_pattern((OSQPInt)(nvar-6), nvar, me, mi, nb, impl_->Ap, impl_->Ai);
    impl_->nnzA = (OSQPInt)(me*nvar + 2*(nvar-6) + nvar);
    fill_A_values(p.Aeq, (OSQPInt)(nvar-6), nvar, me, mi, nb, impl_->Ax);

    // q,l,u
    impl_->q.resize((size_t)nvar);
    for (OSQPInt i=0;i<nvar;++i) impl_->q[(size_t)i]=(OSQPFloat)p.c((int)i);
    fill_lu(p.beq, p.bineq, p.lb, p.ub, me, mi, nb, impl_->l, impl_->u);

    // Wrap and setup
    OSQPCscMatrix* P0 = OSQPCscMatrix_new(nvar, nvar, impl_->nnzP,
                                          impl_->Px.data(), impl_->Pi.data(), impl_->Pp.data());
    OSQPCscMatrix* A0 = OSQPCscMatrix_new(impl_->m, nvar, impl_->nnzA,
                                          impl_->Ax.data(), impl_->Ai.data(), impl_->Ap.data());
    OSQPSolver* solver = nullptr;
    OSQPInt status = osqp_setup(&solver,
                                P0, impl_->q.data(),
                                A0, impl_->l.data(), impl_->u.data(),
                                impl_->m, nvar,
                                impl_->settings);
    // Free wrappers
    if (P0) OSQPCscMatrix_free(P0);
    if (A0) OSQPCscMatrix_free(A0);
    if (status != 0 || !solver) {
      if (impl_->verbose) std::cerr << "[OSQP] setup failed: " << status << "\n";
      r.ok = false; return r;
    }
    impl_->solver = solver;
    impl_->initialized = true;
  } else {
    // Update P (diagonal may change), A eq block values, q, l, u
    fill_P_diag(p.Q, impl_->Px);
    fill_A_values(p.Aeq, (OSQPInt)(nvar-6), nvar, me, mi, nb, impl_->Ax);
    for (OSQPInt i=0;i<nvar;++i) impl_->q[(size_t)i]=(OSQPFloat)p.c((int)i);
    fill_lu(p.beq, p.bineq, p.lb, p.ub, me, mi, nb, impl_->l, impl_->u);

    osqp_update_data_vec(impl_->solver,
                         impl_->q.data(),
                         impl_->l.data(),
                         impl_->u.data());
    osqp_update_data_mat(impl_->solver,
                         impl_->Px.data(), nullptr, impl_->nnzP,
                         impl_->Ax.data(), nullptr, impl_->nnzA);
  }

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
  r.x.resize((int)nvar);
  if (sol && sol->x) for (int i=0;i<(int)nvar;++i) r.x(i) = (double)sol->x[i];
  return r;
}
