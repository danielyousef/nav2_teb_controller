#pragma once

// Shared utilities for verifying analytic edge Jacobians against
// finite-difference (numeric) differentiation over the g2o oplus manifold.

#include <gtest/gtest.h>

#include <g2o/core/jacobian_workspace.h>
#include <g2o/core/optimizable_graph.h>

#include <Eigen/Core>

#include <cstddef>
#include <vector>

namespace nav2_teb_controller {

// Exposes the (protected) analytic Jacobian maps of an edge so that tests can
// read them after calling edge.linearizeOplus(workspace).
//
// Member functions of a class template are instantiated lazily, so only the
// accessor(s) matching the concrete edge kind are ever compiled.
template <typename EdgeT>
class TestEdgeJacobian : public EdgeT {
 public:
  const Eigen::MatrixXd jacobian(size_t i) const {
    return this->_jacobianOplus[i];
  }
  const Eigen::MatrixXd jacobianXi() const {
    return this->_jacobianOplusXi;
  }
  const Eigen::MatrixXd jacobianXj() const {
    return this->_jacobianOplusXj;
  }
};

// Runs the analytic linearizeOplus() through a g2o workspace (so the Jacobian
// maps are backed by allocated memory) and returns a copy of each block.
template <typename EdgeT>
std::vector<Eigen::MatrixXd> runAnalyticJacobians(EdgeT & edge) {
  g2o::JacobianWorkspace ws;
  ws.updateSize(&edge);
  ws.allocate();
  edge.linearizeOplus(ws);

  const size_t nv = edge.vertices().size();
  std::vector<Eigen::MatrixXd> result(nv);
  auto & readable = static_cast<TestEdgeJacobian<EdgeT> &>(edge);
  for (size_t i = 0; i < nv; ++i)
    result[i] = readable.jacobian(i);
  return result;
}

// Central-difference FD of the error over the oplus manifold of every vertex
// (skipping fixed vertices), then compares against the analytic blocks.
template <typename EdgeT>
void expectAnalyticJacobianMatchesNumeric(EdgeT & edge,
                                          std::vector<Eigen::MatrixXd> analytic,
                                          double tol = 2e-4) {
  const size_t nv = edge.vertices().size();
  const double delta = 1e-6;

  ASSERT_EQ(analytic.size(), nv);
  for (size_t i = 0; i < nv; ++i) {
    auto * vertex = static_cast<g2o::OptimizableGraph::Vertex *>(edge.vertex(i));
    if (vertex->fixed())
      continue;
    const int dim = vertex->dimension();
    ASSERT_EQ(analytic[i].rows(), static_cast<int>(edge.error().size()));
    ASSERT_EQ(analytic[i].cols(), dim);

    Eigen::MatrixXd numeric = Eigen::MatrixXd::Zero(analytic[i].rows(), dim);
    std::vector<double> update(dim, 0.0);
    for (int d = 0; d < dim; ++d) {
      update[d] = delta;
      vertex->push();
      vertex->oplus(update.data());
      edge.computeError();
      const Eigen::VectorXd e_plus = edge.error();
      vertex->pop();

      update[d] = -delta;
      vertex->push();
      vertex->oplus(update.data());
      edge.computeError();
      const Eigen::VectorXd e_minus = edge.error();
      vertex->pop();
      update[d] = 0.0;

      numeric.col(d) = (e_plus - e_minus) / (2.0 * delta);
    }

    for (int r = 0; r < numeric.rows(); ++r) {
      for (int c = 0; c < numeric.cols(); ++c) {
        EXPECT_NEAR(analytic[i](r, c), numeric(r, c), tol)
            << "vertex " << i << " block (" << r << "," << c << ")"
            << " analytic=" << analytic[i](r, c)
            << " numeric=" << numeric(r, c);
      }
    }
  }
}

// Convenience wrappers for the three edge arities.
template <typename EdgeT>
void expectAnalyticJacobianMatchesNumericMulti(EdgeT & edge, double tol = 2e-4) {
  expectAnalyticJacobianMatchesNumeric(edge, runAnalyticJacobians(edge), tol);
}

template <typename EdgeT>
void expectAnalyticJacobianMatchesNumericUnary(EdgeT & edge, double tol = 2e-4) {
  g2o::JacobianWorkspace ws;
  ws.updateSize(&edge);
  ws.allocate();
  edge.linearizeOplus(ws);
  auto & readable = static_cast<TestEdgeJacobian<EdgeT> &>(edge);
  std::vector<Eigen::MatrixXd> analytic{readable.jacobianXi()};
  expectAnalyticJacobianMatchesNumeric(edge, analytic, tol);
}

template <typename EdgeT>
void expectAnalyticJacobianMatchesNumericBinary(EdgeT & edge,
                                                double tol = 2e-4) {
  g2o::JacobianWorkspace ws;
  ws.updateSize(&edge);
  ws.allocate();
  edge.linearizeOplus(ws);
  auto & readable = static_cast<TestEdgeJacobian<EdgeT> &>(edge);
  std::vector<Eigen::MatrixXd> analytic{readable.jacobianXi(),
                                        readable.jacobianXj()};
  expectAnalyticJacobianMatchesNumeric(edge, analytic, tol);
}

}  // namespace nav2_teb_controller
