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
    typedef double Scalar;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::MatrixXd Matrix;

    typedef Eigen::Matrix<Scalar,3,1> Vector3;
    typedef Eigen::Matrix<Scalar,6,1> Vector6;
    typedef Eigen::Matrix<Scalar,3,Eigen::Dynamic> Matrix3x;

    typedef Eigen::Ref<Eigen::VectorXd> RefVector;
    typedef const Eigen::Ref<const Eigen::VectorXd>& ConstRefVector;
    typedef Eigen::Ref<Eigen::MatrixXd> RefMatrix;
    typedef const Eigen::Ref<const Eigen::MatrixXd> ConstRefMatrix;

    typedef std::size_t Index;

    /**
     * Convert the input SE3 object to a 7D vector of floats [X,Y,Z,Q1,Q2,Q3,Q4].
     */
    void se3ToXYZQUAT(const se3::SE3 & M, RefVector xyzQuat);

    /**
     * Convert the input SE3 object to a 12D vector of floats [X,Y,Z,R11,R12,R13,R14,...].
     */
    void se3ToVector(const se3::SE3 & M, RefVector vec);

    void vectorToSE3(RefVector vec, se3::SE3 & M);

    void errorInSE3 (const se3::SE3 & M,
                     const se3::SE3 & Mdes,
                     se3::Motion & error);


    void pseudoInverse(ConstRefMatrix A,
                       RefMatrix Apinv,
                       double tolerance,
                       unsigned int computationOptions = Eigen::ComputeThinU | Eigen::ComputeThinV);

    void pseudoInverse(ConstRefMatrix A,
                       Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                       RefMatrix Apinv,
                       double tolerance,
                       unsigned int computationOptions);

    void pseudoInverse(ConstRefMatrix A,
                       Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                       RefMatrix Apinv,
                       double tolerance,
                       double * nullSpaceBasisOfA,
                       int &nullSpaceRows,
                       int &nullSpaceCols,
                       unsigned int computationOptions);

    void dampedPseudoInverse(ConstRefMatrix A,
                             Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                             RefMatrix Apinv,
                             double tolerance,
                             double dampingFactor,
                             unsigned int computationOptions = Eigen::ComputeThinU | Eigen::ComputeThinV,
                             double * nullSpaceBasisOfA=0,
                             int *nullSpaceRows=0, int *nullSpaceCols=0);

    void nullSpaceBasisFromDecomposition(Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                                         double tolerance,
                                         double * nullSpaceBasisMatrix,
                                         int &rows, int &cols);

    template<typename Derived>
    inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
    {
      return ( (x - x).array() == (x - x).array()).all();
    }

  }
}

#endif // ifndef __invdyn_math_utils_hpp__
