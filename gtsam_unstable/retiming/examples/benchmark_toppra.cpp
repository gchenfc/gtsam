#include <memory>
#include <gtsam/base/timing.h>
#include <toppra/algorithm.hpp>
#include <toppra/algorithm/toppra.hpp>
#include <toppra/constraint.hpp>
#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>
#ifdef BUILD_WITH_qpOASES
#include <toppra/solver/qpOASES-wrapper.hpp>
#endif
#ifdef BUILD_WITH_GLPK
#include <toppra/solver/glpk-wrapper.hpp>
#endif
#include <toppra/solver/seidel.hpp>
#include <toppra/toppra.hpp>

// using Solver = toppra::solver::qpOASESWrapper;
// using Solver = toppra::solver::GLPKWrapper;
using Solver = toppra::solver::Seidel;

using toppra::Bounds;
using toppra::GeometricPath;
using toppra::LinearConstraint;
using toppra::Matrices;
using toppra::Matrix;
using toppra::Vector;
using toppra::Vectors;

/// A Joint Acceleration Constraint class.
class MyConstraint : public LinearConstraint {
 public:
  MyConstraint()
      : LinearConstraint(1,      // num constraints
                         1,      // dim of v
                         true,   // constantF
                         false,  // has u bound
                         false   // has x bound
        ) {}

 private:
  //  * \f{eqnarray}
  //  *     \mathbf a_i u + \mathbf b_i x + \mathbf c_i &= v \\
//  *     \mathbf F_i v & \leq \mathbf g_i                \\
//  *     x^b_{i, 0} \leq x & \leq x^b_{i, 1} \\
//  *     u^b_{i, 0} \leq u & \leq u^b_{i, 1}
  //  * \f}
  void computeParams_impl(const GeometricPath& path, const Vector& gridpoints,
                          Vectors& a, Vectors& b, Vectors& c, Matrices& F,
                          Vectors& g, Bounds& ubound, Bounds& xbound) override {
    Eigen::Index N_1 = gridpoints.size();
    // 1u + 1x = v
    // 1v <= 0.1
    Matrix& _F = F[0];  // constantF so F & g should only have 1 element
    Vector& _g = g[0];
    _F << 1;
    _g << 0.1;
    for (int i = 0; i < N_1; ++i) {
      a[i] << 1;
      b[i] << 1;
      c[i] << 0;
    }
  }
};  // class MyConstraint

class ProblemInstance {
 public:
  ProblemInstance(int N = 10000) {
    /*
  RetimingFactorGraph factors;
  for (int i = 0; i < N; ++i) {
    factors.push_back(RetimingFactor::Equality({X(i + 1), X(i), U(i)},
                                               Create({-1.0, 1.0, 0.5}, 0.0)));
    factors.push_back(
        RetimingFactor::Inequality({U(i), X(i)}, Create({1, 1}, 0.1)));
  }
  factors.push_back(RetimingFactor::Equality({X(0)}, Create({1}, 1.0)));
    */
    /*
      x' = x + (0.5).u = x + 2.delta.u, so delta = 0.25
      (x + u) <= 0.1
      x0 = 1.0
    */
    double delta = 0.25;
    toppra::Matrix coeff{2, 1};  // Linear
    coeff << 1, 0;               // straight line to t = N
    path = std::make_shared<toppra::PiecewisePolyPath>(
        Matrices{coeff}, std::vector<double>{0, N * delta});
    v = toppra::LinearConstraintPtrs{std::make_shared<MyConstraint>()};
  };

  std::shared_ptr<toppra::PiecewisePolyPath> path;
  toppra::LinearConstraintPtrs v;
  int nDof = 2;
};

#define ASSERT_TRUE(x) assert(x)
#define ASSERT_EQ(x, y) assert(x == y)
#define ASSERT_DOUBLES_EQ(x, y) assert(std::abs(x - y) < 1e-6)
#define ASSERT_GE(x, y) assert(x >= y)

void run(int N = 10000) {
  ProblemInstance instance(N);
  const auto& input_path = *instance.path;

  toppra::algorithm::TOPPRA problem{instance.v, instance.path};
  problem.setN(N);
  problem.solver(std::make_shared<Solver>());

  for (int i = 0; i < 10; ++i) {
    gttic_(TOPP);
    auto ret_code = problem.computePathParametrization(1, 0.1);
    ASSERT_TRUE(ret_code);
  }
  gtsam::tictoc_print_();

  const auto& sol = problem.getParameterizationData();

  TOPPRA_LOG_DEBUG("Pre constructed");
  toppra::parametrizer::ConstAccel output_traj{instance.path, sol.gridpoints,
                                               sol.parametrization};
  ASSERT_TRUE(output_traj.validate());

  // These come from QOPP
  ASSERT_DOUBLES_EQ(1, sol.parametrization(0));
  ASSERT_DOUBLES_EQ(0.55, sol.parametrization(1));
  ASSERT_DOUBLES_EQ(0.325, sol.parametrization(2));
  ASSERT_DOUBLES_EQ(0.2125, sol.parametrization(3));
  ASSERT_DOUBLES_EQ(0.15625, sol.parametrization(4));
  ASSERT_DOUBLES_EQ(0.128125, sol.parametrization(5));
  ASSERT_DOUBLES_EQ(0.114062, sol.parametrization(6));
  ASSERT_DOUBLES_EQ(0.107031, sol.parametrization(7));
  ASSERT_DOUBLES_EQ(0.103516, sol.parametrization(8));
  ASSERT_DOUBLES_EQ(0.101758, sol.parametrization(9));
  ASSERT_DOUBLES_EQ(0.100879, sol.parametrization(10));
  ASSERT_DOUBLES_EQ(0.100439, sol.parametrization(11));
  ASSERT_DOUBLES_EQ(0.10022, sol.parametrization(12));
  ASSERT_DOUBLES_EQ(0.10011, sol.parametrization(13));
  ASSERT_DOUBLES_EQ(0.100055, sol.parametrization(14));
  ASSERT_DOUBLES_EQ(0.100027, sol.parametrization(15));
  ASSERT_DOUBLES_EQ(0.100014, sol.parametrization(16));
  for (int i = 20; i < N; ++i) {
    ASSERT_DOUBLES_EQ(0.1, sol.parametrization(i));
  }

  // std::cout << sol.parametrization << std::endl;
  // std::cout << "Gridpoints: " << sol.gridpoints.transpose() << std::endl;
  // std::cout << "Parameterization: " << sol.parametrization.transpose()
  //           << std::endl;
}

int main() {
  for (int N = 1000; N <= 30000; N += 1000) {
    std::cout << "Running with N = " << N << std::endl;
    run(N);
  }
}
