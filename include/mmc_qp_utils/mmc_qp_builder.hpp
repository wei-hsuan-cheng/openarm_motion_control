#pragma once
#include <Eigen/Dense>
#include "iqpsolver.hpp"

// Parameters driving the MMC-QP construction
struct MMCParams {
  double lambda_q    = 1e-2;   // weight on joint velocities
  double lambda_delta_min = 1e-3;
  double lambda_delta_max = 1e+2;

  // Slack scaling T = diag(t_lin*I3, t_ang*I3) used in equality [J  T] [qdot; delta] = nu
  double t_lin = 1.0; // scale for translational slack
  double t_ang = 1.0; // scale for angular slack

  // delta (slack on task-space equality) bounds
  Eigen::VectorXd delta_min;   // size 6
  Eigen::VectorXd delta_max;   // size 6

  // Joint velocity bounds (fallback if not using YAML velocity column)
  Eigen::VectorXd qd_min;      // size n
  Eigen::VectorXd qd_max;      // size n

  // Velocity damper near joint limits
  bool   use_joint_limit_damper = true;
  double eta = 1.0;                 // scale
  double rho_i = M_PI * 50.0/180.0; // inner distance (rad)
  double rho_s = M_PI *  2.0/180.0; // stop distance (rad)

  // Sign for linear manipulability term in objective:
  // minimize( ... + c^T x ), c=head(n)=jw_sign * Jw, with jw_sign=-1 => maximize w
  int jw_sign = -1;
};

// Utility: clamp scalar to [a,b]
inline double clamp(double v, double a, double b) {
  return std::max(a, std::min(b, v));
}

class MMCQPBuilder {
public:
  explicit MMCQPBuilder(const MMCParams& P) : P_(P) {}

  // Build QP with decision x = [qdot(n); delta(6)]
  // J: 6xn, nu: 6, Jw: n, q: n, joint_limits: n x 4 [ll, ul, vel, effort]
  QPProblem build(const Eigen::MatrixXd& J,
                  const Eigen::VectorXd& nu,
                  const Eigen::VectorXd& Jw,
                  const Eigen::VectorXd& q,
                  const Eigen::MatrixXd& joint_limits,
                  double lambda_delta_hint = 1.0,
                  double Q_c_ratio = 1.0) const
  {
    const int n = J.cols();
    const int nv = n + 6; // qdot + delta
    QPProblem prob;

    // Q
    prob.Q.setZero(nv, nv);
    prob.Q.topLeftCorner(n, n).diagonal().array() = P_.lambda_q;
    const double lam_d = clamp(lambda_delta_hint, P_.lambda_delta_min, P_.lambda_delta_max);
    prob.Q.bottomRightCorner(6, 6).diagonal().array() = lam_d;
    prob.Q *= Q_c_ratio;

    // c
    prob.c = Eigen::VectorXd::Zero(nv);
    prob.c.head(n) = P_.jw_sign * Jw; // maximize m by minimizing c^T x with negative sign

    // Equality: [J  T] [qdot; delta] = nu, where T = diag(t_lin*I3, t_ang*I3)
    prob.Aeq.resize(6, nv);
    Eigen::Matrix<double,6,6> T = Eigen::Matrix<double,6,6>::Zero();
    T.topLeftCorner(3,3).setIdentity();
    T.topLeftCorner(3,3) *= P_.t_lin;
    T.bottomRightCorner(3,3).setIdentity();
    T.bottomRightCorner(3,3) *= P_.t_ang;
    prob.Aeq << J, T;
    prob.beq = nu;

    // Bounds: lb <= x <= ub
    prob.lb = Eigen::VectorXd::Constant(nv, -std::numeric_limits<double>::infinity());
    prob.ub = Eigen::VectorXd::Constant(nv,  std::numeric_limits<double>::infinity());

    // Joint velocity bounds
    if (joint_limits.rows()==n && joint_limits.cols()>=3) {
      // Use velocity column (abs limit or signed limits as provided)
      Eigen::VectorXd vlim = joint_limits.col(2).array().abs().matrix();
      prob.lb.head(n) = -vlim;
      prob.ub.head(n) =  vlim;
    } else if (P_.qd_min.size()==n && P_.qd_max.size()==n) {
      prob.lb.head(n) = P_.qd_min;
      prob.ub.head(n) = P_.qd_max;
    } // else leave unbounded (not recommended)

    // Delta bounds
    if (P_.delta_min.size()==6 && P_.delta_max.size()==6) {
      prob.lb.tail(6) = P_.delta_min;
      prob.ub.tail(6) = P_.delta_max;
    } else {
      prob.lb.tail(6).array() = -0.2;
      prob.ub.tail(6).array() =  0.2;
    }

    // Inequalities from velocity damper near joint limits (fixed 2n rows)
    // Row layout per joint i: lower (-qdot_i <= rhs_lower), upper (qdot_i <= rhs_upper)
    {
      const double kInf = 1e30; // large to deactivate when far from limits
      prob.Aineq.setZero(2*n, nv);
      prob.bineq.setZero(2*n);
      for (int i=0;i<n;++i) {
        // Structure constant
        prob.Aineq(2*i + 0, i) = -1.0;
        prob.Aineq(2*i + 1, i) = +1.0;

        double rhs_lower = kInf;
        double rhs_upper = kInf;
        if (joint_limits.rows()==n && joint_limits.cols()>=2 && P_.use_joint_limit_damper) {
          const double ll = joint_limits(i,0);
          const double ul = joint_limits(i,1);
          const double dist_ll = q(i) - ll;
          const double dist_ul = ul - q(i);
          auto compute_rhs = [&](double rho){
            if (rho < P_.rho_i) {
              return P_.eta * ((rho - P_.rho_s) / (P_.rho_i - P_.rho_s));
            } else {
              return kInf;
            }
          };
          rhs_lower = compute_rhs(dist_ll);
          rhs_upper = compute_rhs(dist_ul);
        }
        prob.bineq(2*i + 0) = rhs_lower;
        prob.bineq(2*i + 1) = rhs_upper;
      }
    }

    return prob;
  }

private:
  MMCParams P_;
};
