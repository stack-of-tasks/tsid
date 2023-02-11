//
// Copyright (c) 2022 INRIA
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
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
