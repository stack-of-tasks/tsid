//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// PinInvDyn is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// PinInvDyn If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_math_utils_hpp__
#define __invdyn_math_utils_hpp__

#include <Eigen/Core>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace pininvdyn
{
  namespace math
  {
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;
    typedef Eigen::Ref<Eigen::VectorXd> RefVector;
    typedef const Eigen::Ref<const Eigen::VectorXd>& ConstRefVector;
    typedef Eigen::Ref<Eigen::MatrixXd> RefMatrix;
    typedef const Eigen::Ref<const Eigen::MatrixXd> ConstRefMatrix;

    typedef std::size_t Index;

    /**
     * Convert the input SE3 object to a 7D tuple of floats [X,Y,Z,Q1,Q2,Q3,Q4].
     */
    static void se3ToXYZQUAT(const se3::SE3 & M, RefVector xyzQuat)
    {
      assert(xyzQuat.size()>=7);
      xyzQuat.head<3>() = M.translation();
      xyzQuat.tail<4>() = Eigen::Quaterniond(M.rotation()).coeffs();
    }

    static void se3ToVector(const se3::SE3 & M, RefVector vec)
    {
      assert(vec.size()>=12);
      vec.head<3>() = M.translation();
      typedef Eigen::Matrix<double,9,1> Vector9;
      vec.tail<9>() = Eigen::Map<const Vector9>(&M.rotation()(0), 9);
    }

    static void vectorToSE3(RefVector vec, se3::SE3 & M)
    {
      assert(vec.size()>=12);
      M.translation( vec.head<3>() );
      typedef Eigen::Matrix<double,3,3> Matrix3;
      M.rotation( Eigen::Map<const Matrix3>(&vec(3), 3, 3) );
    }

    static void errorInSE3 (const se3::SE3 & M,
                            const se3::SE3 & Mdes,
                            se3::Motion & error)
    {
      error = se3::log6(Mdes.inverse() * M);
    }

  }
}

#endif // ifndef __invdyn_math_utils_hpp__
