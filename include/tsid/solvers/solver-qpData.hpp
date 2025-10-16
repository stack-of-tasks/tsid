//
// Copyright (c) 2022 INRIA
//

#ifndef __solvers_qpdata_hpp__
#define __solvers_qpdata_hpp__

namespace tsid {
namespace solvers {

template <typename scalar_>
struct QPDataBaseTpl {
  typedef Eigen::Matrix<scalar_, Eigen::Dynamic, 1> Vector;
  typedef Eigen::Matrix<scalar_, Eigen::Dynamic, Eigen::Dynamic> Matrix;

  Matrix H;  // cost to minimize
  Vector g;
  Matrix CE;  // equality constraints
  Vector ce0;
};

template <typename scalar_>
struct QPDataTpl : QPDataBaseTpl<scalar_> {
  typedef Eigen::Matrix<scalar_, Eigen::Dynamic, 1> Vector;
  typedef Eigen::Matrix<scalar_, Eigen::Dynamic, Eigen::Dynamic> Matrix;

  Matrix CI;     // inequality constraints
  Vector ci_lb;  // lower bound
  Vector ci_ub;  // upper bound
};

template <typename scalar_>
struct QPDataQuadProgTpl : QPDataBaseTpl<scalar_> {
  typedef Eigen::Matrix<scalar_, Eigen::Dynamic, 1> Vector;
  typedef Eigen::Matrix<scalar_, Eigen::Dynamic, Eigen::Dynamic> Matrix;

  Matrix CI;   // inequality constraints, one-sided
  Vector ci0;  // stack of lower and upper bounds
};
}  // namespace solvers
}  // namespace tsid

#endif  // ifndef __solvers_qpdata_hpp__
