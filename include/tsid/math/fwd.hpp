//
// Copyright (c) 2017 CNRS
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

#ifndef __invdyn_math_fwd_hpp__
#define __invdyn_math_fwd_hpp__

#include <Eigen/Core>

#ifdef EIGEN_RUNTIME_NO_MALLOC
  #define EIGEN_MALLOC_ALLOWED Eigen::internal::set_is_malloc_allowed(true);
  #define EIGEN_MALLOC_NOT_ALLOWED Eigen::internal::set_is_malloc_allowed(false);
#else
  #define EIGEN_MALLOC_ALLOWED
  #define EIGEN_MALLOC_NOT_ALLOWED 
#endif

namespace tsid
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
    
    typedef Eigen::Ref<Vector3>             RefVector3;
    typedef const Eigen::Ref<const Vector3> ConstRefVector3;
    
    typedef Eigen::Ref<Vector>              RefVector;
    typedef const Eigen::Ref<const Vector>  ConstRefVector;
    
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
