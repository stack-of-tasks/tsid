//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// pinocchio If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_math_fwd_hpp__
#define __invdyn_math_fwd_hpp__

#include <Eigen/Core>

namespace pininvdyn
{
  namespace math
  {
    
    typedef double Scalar;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> Vector;
    typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> Matrix;
    typedef Eigen::VectorXi VectorXi;
    typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;
    
    typedef Eigen::Matrix<Scalar,3,1> Vector3;
    typedef Eigen::Matrix<Scalar,6,1> Vector6;
    typedef Eigen::Matrix<Scalar,3,Eigen::Dynamic> Matrix3x;
    
    typedef Eigen::Ref<Vector>              RefVector;
    typedef const Eigen::Ref<const Vector> & ConstRefVector;
    typedef Eigen::Ref<Matrix>              RefMatrix;
    typedef const Eigen::Ref<const Matrix>  ConstRefMatrix;
    
    typedef std::size_t Index;
    
    // Forward declaration of constraints
    class ConstraintBase;
    class ConstraintEquality;
    class ConstraintInequality;
    class ConstraintBound;
    
  }
}

#endif // ifndef __invdyn_math_fwd_hpp__
