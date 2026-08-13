// src/obstacle_map_2d.cpp
#include <cassert>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_teb_controller/obstacles/esdf.hpp>

namespace nav2_teb_controller {

// ─────────────────────────────────────────────────────────────────────────────
// ROS2 wrapper
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleMap2D::update(const nav2_costmap_2d::Costmap2D &costmap) {
  update(costmap.getCharMap(), costmap.getSizeInCellsX(), costmap.getSizeInCellsY(),
         costmap.getResolution(), costmap.getOriginX(), costmap.getOriginY(),
         nav2_costmap_2d::LETHAL_OBSTACLE);  // = 254
}

// ─────────────────────────────────────────────────────────────────────────────
// Core update
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleMap2D::update(const uint8_t *data, unsigned nx, unsigned ny, double res,
                           double origin_x, double origin_y, uint8_t lethal_threshold) {
  assert(data != nullptr);
  assert(nx > 0 && ny > 0);
  assert(res > 0.0);

  nx_ = nx;
  ny_ = ny;
  res_ = res;
  origin_x_ = origin_x;
  origin_y_ = origin_y;

  const size_t n = static_cast<size_t>(nx) * ny;
  edt_.resize(n);
  g_.resize(n);

  // Build binary occupancy (true = obstacle)
  std::vector<bool> occ(n);
  for (size_t i = 0; i < n; ++i)
    occ[i] = (data[i] >= lethal_threshold);

  phase1_columnScan(occ);
  phase2_rowScan();
}

// ─────────────────────────────────────────────────────────────────────────────
// Meijster EDT — Phase 1: column scan (Y direction)
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleMap2D::phase1_columnScan(const std::vector<bool> &occ) {
  const auto INF = static_cast<double>(ny_ * ny_ + 1);

  for (unsigned x = 0; x < nx_; ++x) {
    // Forward pass
    g_[idx(x, 0)] = occ[idx(x, 0)] ? 0.0 : INF;
    for (unsigned y = 1; y < ny_; ++y) {
      g_[idx(x, y)] = occ[idx(x, y)] ? 0.0 : g_[idx(x, y - 1)] + 1.0;
    }
    // Backward pass
    for (int y = static_cast<int>(ny_) - 2; y >= 0; --y) {
      double back = g_[idx(x, y + 1)] + 1.0;
      if (back < g_[idx(x, y)])
        g_[idx(x, y)] = back;
    }
  }
  // g_ now holds: min distance in Y direction (in cells) for each (x,y)
}

// ─────────────────────────────────────────────────────────────────────────────
// Meijster EDT — Phase 2: row scan (X direction, parabola envelope)
// ─────────────────────────────────────────────────────────────────────────────

void ObstacleMap2D::phase2_rowScan() {
  std::vector<int> s(nx_);       // parabola centers
  std::vector<int> t(nx_);       // separation points
  std::vector<double> row(nx_);  // g values for current row

  for (unsigned y = 0; y < ny_; ++y) {
    for (unsigned x = 0; x < nx_; ++x)
      row[x] = g_[idx(x, y)];

    // f(x, i) = squared distance from x to parabola at i
    auto f = [&](int x, int i) -> double {
      double gi = row[i];
      return static_cast<double>((x - i) * (x - i)) + gi * gi;
    };

    // sep(i, u): separation point between parabola i and u
    auto sep = [&](int i, int u) -> int {
      double gi = row[i], gu = row[u];
      double num = static_cast<double>(u * u - i * i) + gu * gu - gi * gi;
      double den = 2.0 * static_cast<double>(u - i);
      return static_cast<int>(num / den);
    };

    int q = 0;
    s[0] = 0;
    t[0] = 0;

    // Build lower envelope of parabolas
    for (unsigned u = 1; u < nx_; ++u) {
      while (q >= 0 && f(t[q], s[q]) > f(t[q], static_cast<int>(u)))
        --q;
      if (q < 0) {
        q = 0;
        s[0] = static_cast<int>(u);
      } else {
        int w = 1 + sep(s[q], static_cast<int>(u));
        if (w < static_cast<int>(nx_)) {
          ++q;
          s[q] = static_cast<int>(u);
          t[q] = w;
        }
      }
    }

    // Scan right to left, filling final EDT
    for (int u = static_cast<int>(nx_) - 1; u >= 0; --u) {
      edt_[idx(static_cast<unsigned>(u), y)] = static_cast<float>(std::sqrt(f(u, s[q])) * res_);
      if (u == t[q])
        --q;
    }
  }
}

// ─────────────────────────────────────────────────────────────────────────────
// Query: bilinear interpolation + gradient
// ─────────────────────────────────────────────────────────────────────────────

ObstacleMap2D::InterpResult ObstacleMap2D::interpolate(double wx, double wy) const {
  if (!isInitialized())
    return {kOutOfBoundsDist, 0.0, 0.0};

  // World → fractional grid coordinates (cell centers at integer coords)
  double gx = (wx - origin_x_) / res_ - 0.5;
  double gy = (wy - origin_y_) / res_ - 0.5;

  // Out-of-bounds guard
  if (gx < 0.0 || gy < 0.0 || gx >= static_cast<double>(nx_ - 1) ||
      gy >= static_cast<double>(ny_ - 1)) {
    // Clamp and return boundary value with zero gradient
    // (optimizer can't push further anyway)
    gx = std::clamp(gx, 0.0, static_cast<double>(nx_ - 2));
    gy = std::clamp(gy, 0.0, static_cast<double>(ny_ - 2));
  }

  const int ix = static_cast<int>(gx);
  const int iy = static_cast<int>(gy);
  const double tx = gx - ix;  // [0, 1)
  const double ty = gy - iy;  // [0, 1)

  // Bilinear interpolation of 4 neighbors
  const double d00 = edt_[idx(ix, iy)];
  const double d10 = edt_[idx(ix + 1, iy)];
  const double d01 = edt_[idx(ix, iy + 1)];
  const double d11 = edt_[idx(ix + 1, iy + 1)];

  const double d =
      (1 - tx) * (1 - ty) * d00 + tx * (1 - ty) * d10 + (1 - tx) * ty * d01 + tx * ty * d11;

  // Analytical gradient of bilinear interpolation
  // ∂d/∂wx = (1/res) * ∂d/∂gx
  const double dgx = ((1 - ty) * (d10 - d00) + ty * (d11 - d01)) / res_;
  const double dgy = ((1 - tx) * (d01 - d00) + tx * (d11 - d10)) / res_;

  return {d, dgx, dgy};
}

Eigen::Vector2d ObstacleMap2D::gradient(double wx, double wy) const {
  const InterpResult r = interpolate(wx, wy);
  return Eigen::Vector2d(r.dgx, r.dgy);
}

double ObstacleMap2D::queryDistance(double wx, double wy) const {
  return interpolate(wx, wy).d;
}

ObstacleMap2D::QueryResult ObstacleMap2D::query(double wx, double wy) const {
  const InterpResult r = interpolate(wx, wy);

  // Normalize to unit vector — satisfies Eikonal |∇d| = 1
  // Guard against zero-gradient (exactly on obstacle boundary)
  Eigen::Vector2d grad(r.dgx, r.dgy);
  const double grad_norm = grad.norm();
  if (grad_norm > 1e-6)
    grad /= grad_norm;
  else
    grad = Eigen::Vector2d::Zero();

  return {r.d, grad};
}

}  // namespace nav2_teb_controller
