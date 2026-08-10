#pragma once

// Shared math for the kinodynamics edges (velocity / acceleration / jerk).
// A "segment" connects pose i -> pose i+1 over time interval dt; the costs
// use the arc-length-adjusted distance and the fast_sigmoid direction term.

#include "nav2_teb_controller/math_utils.hpp"

#include <angles/angles.h>

#include <Eigen/Core>

#include <cmath>

namespace nav2_teb_controller
{

/**
 * @brief Motion quantities of one trajectory segment (pose i -> pose i+1, time dt).
 */
struct SegmentMotion
{
  Eigen::Vector2d delta;    // position difference p_{i+1} - p_i
  double d0 = 0.0;          // Euclidean distance
  double angle_diff = 0.0;  // normalized heading change
  double dist = 0.0;        // distance used by the costs (Euclidean or exact arc)
  double arc = 1.0;         // dist / d0
  double dh_da = 0.0;       // d(arc)/d(angle_diff) for the exact-arc variant
  double dt = 0.0;
  double cos_i = 1.0;       // heading unit vector of pose i
  double sin_i = 0.0;

  /** Raw velocity (distance / dt, no direction term). */
  double velRaw() const { return dist / dt; }

  /** Rotational velocity. */
  double omega() const { return angle_diff / dt; }

  /** Unit vector along delta (zero if degenerate). */
  Eigen::Vector2d unit() const
  {
    return d0 > 1e-12 ? Eigen::Vector2d(delta / d0) : Eigen::Vector2d::Zero();
  }
};

/**
 * @brief Build a segment, optionally replacing the Euclidean distance by the exact arc length.
 */
inline SegmentMotion makeSegmentMotion(
  const Eigen::Vector2d & delta, double theta_i, double theta_ip1, double dt,
  bool exact_arc_length)
{
  SegmentMotion m;
  m.delta = delta;
  m.dt = dt;
  m.cos_i = std::cos(theta_i);
  m.sin_i = std::sin(theta_i);
  m.d0 = delta.norm();
  m.angle_diff = angles::normalize_angle(theta_ip1 - theta_i);
  m.dist = m.d0;
  if (exact_arc_length && m.angle_diff != 0.0 && m.d0 > 1e-12) {
    const double half = m.angle_diff / 2.0;
    const double s_half = std::sin(half);
    const double h = std::abs(m.angle_diff) / (2.0 * std::abs(s_half));
    m.dist = m.d0 * h;
    m.arc = h;
    // h'(a) = sign(a)/(2|sin(a/2)|) - |a|*sign(sin(a/2))*cos(a/2)/(4 sin^2(a/2))
    m.dh_da = std::copysign(1.0, m.angle_diff) / (2.0 * std::abs(s_half)) -
              std::abs(m.angle_diff) * std::copysign(1.0, s_half) * std::cos(half) /
              (4.0 * s_half * s_half);
  }
  return m;
}

/**
 * @brief Direction term sigma(100 * s) with s = delta . (cos theta_i, sin theta_i).
 */
struct SigmoidTerm
{
  double s_dir = 0.0;
  double sigma = 0.0;  // fast_sigmoid(100 * s_dir)
  double deriv = 0.0;  // d sigma / d s_dir
  double rot1 = 0.0;   // d s_dir / d theta_i = -dx sin(theta_i) + dy cos(theta_i)
};

/**
 * @brief Build the direction (sigmoid) term for a segment relative to pose i.
 */
inline SigmoidTerm makeSigmoidTerm(
  const Eigen::Vector2d & delta, double cos_i, double sin_i)
{
  SigmoidTerm st;
  st.s_dir = delta.x() * cos_i + delta.y() * sin_i;
  st.sigma = fast_sigmoid(100.0 * st.s_dir);
  st.deriv = 100.0 / ((1.0 + 100.0 * std::abs(st.s_dir)) * (1.0 + 100.0 * std::abs(st.s_dir)));
  st.rot1 = -delta.x() * sin_i + delta.y() * cos_i;
  return st;
}

/**
 * @brief Error value and partial derivatives of the acceleration over two
 * consecutive segments (poses i, i+1, i+2 with times dt_a, dt_b):
 *   a_lin = 2*(vel_b - vel_a) / (dt_a + dt_b)
 *   a_rot = 2*(omega_b - omega_a) / (dt_a + dt_b)
 * All partials are the exact derivatives of these expressions, including the
 * fast_sigmoid direction term sigma(100 * delta . u_i) and the exact-arc-length
 * variant. The partials are grouped per vertex: lin_pose[k] = (d a_lin/d x,
 * d a_lin/d y, d a_lin/d theta) of pose i+k; lin_dt[k] = d a_lin/d dt of time
 * interval k.
 */
struct AccelPairGrads
{
  double a_lin = 0.0;
  double a_rot = 0.0;
  Eigen::Vector3d lin_pose[3] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                 Eigen::Vector3d::Zero()};
  Eigen::Vector3d rot_pose[3] = {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                                 Eigen::Vector3d::Zero()};
  double lin_dt[2] = {0.0, 0.0};
  double rot_dt[2] = {0.0, 0.0};
};

/**
 * @brief Build the acceleration error and its partial derivatives for a pair
 * of consecutive segments (a = first segment, b = second segment).
 */
inline AccelPairGrads makeAccelPairGrads(
  const SegmentMotion & seg_a, const SigmoidTerm & sig_a, double vel_a, double omega_a,
  const SegmentMotion & seg_b, const SigmoidTerm & sig_b, double vel_b, double omega_b)
{
  AccelPairGrads g;
  const double dta = seg_a.dt;
  const double dtb = seg_b.dt;
  const double sum_time = dta + dtb;
  g.a_lin = 2.0 * (vel_b - vel_a) / sum_time;
  g.a_rot = 2.0 * (omega_b - omega_a) / sum_time;

  const Eigen::Vector2d ua = seg_a.unit();
  const Eigen::Vector2d ub = seg_b.unit();
  // dist_a = d0_a * arc_a, dist_b = d0_b * arc_b
  const double dist_a = seg_a.d0 * seg_a.arc;
  const double dist_b = seg_b.d0 * seg_b.arc;
  // d sigma / d (x, y, theta) of pose i, via s = delta . (cos theta_i, sin theta_i);
  // factor 2 from a_lin = 2*(vel_b - vel_a)/sum
  const double c1 = 2.0 * dist_a * sig_a.deriv / (sum_time * dta);
  const double c2 = 2.0 * dist_b * sig_b.deriv / (sum_time * dtb);
  // arc-length contribution: d dist / d theta_i, theta_{i+1}, theta_{i+2}
  const double arc_a = sig_a.sigma * seg_a.d0 * seg_a.dh_da / (sum_time * dta);
  const double arc_b = sig_b.sigma * seg_b.d0 * seg_b.dh_da / (sum_time * dtb);

  // pose i (first vertex of segment a)
  g.lin_pose[0] = Eigen::Vector3d(
    2.0 * sig_a.sigma * seg_a.arc * ua.x() / (sum_time * dta) + c1 * seg_a.cos_i,
    2.0 * sig_a.sigma * seg_a.arc * ua.y() / (sum_time * dta) + c1 * seg_a.sin_i,
    2.0 * arc_a - 2.0 * dist_a * sig_a.deriv * sig_a.rot1 / (sum_time * dta));
  // pose i+1 (shared)
  g.lin_pose[1] = Eigen::Vector3d(
    -2.0 / sum_time * (sig_b.sigma * seg_b.arc * ub.x() / dtb +
                       sig_a.sigma * seg_a.arc * ua.x() / dta) -
      (c2 * seg_b.cos_i + c1 * seg_a.cos_i),
    -2.0 / sum_time * (sig_b.sigma * seg_b.arc * ub.y() / dtb +
                       sig_a.sigma * seg_a.arc * ua.y() / dta) -
      (c2 * seg_b.sin_i + c1 * seg_a.sin_i),
    2.0 * (dist_b * sig_b.deriv * sig_b.rot1 / (sum_time * dtb) - arc_b - arc_a));
  // pose i+2 (second vertex of segment b)
  g.lin_pose[2] = Eigen::Vector3d(
    2.0 * sig_b.sigma * seg_b.arc * ub.x() / (sum_time * dtb) + c2 * seg_b.cos_i,
    2.0 * sig_b.sigma * seg_b.arc * ub.y() / (sum_time * dtb) + c2 * seg_b.sin_i,
    2.0 * arc_b);

  // rotational acceleration partials (normalize_angle is C1 away from +-pi)
  g.rot_pose[0] = Eigen::Vector3d(0.0, 0.0, 2.0 / (sum_time * dta));
  g.rot_pose[1] = Eigen::Vector3d(
    0.0, 0.0, -2.0 / sum_time * (1.0 / dta + 1.0 / dtb));
  g.rot_pose[2] = Eigen::Vector3d(0.0, 0.0, 2.0 / (sum_time * dtb));

  // time interval partials: a = 2*(v_b - v_a)/sum, v = dist/dt
  g.lin_dt[0] = 2.0 / sum_time * vel_a / dta - g.a_lin / sum_time;
  g.lin_dt[1] = -2.0 / sum_time * vel_b / dtb - g.a_lin / sum_time;
  g.rot_dt[0] = 2.0 / sum_time * omega_a / dta - g.a_rot / sum_time;
  g.rot_dt[1] = -2.0 / sum_time * omega_b / dtb - g.a_rot / sum_time;
  return g;
}

}  // namespace nav2_teb_controller
