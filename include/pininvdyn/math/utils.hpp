//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// PinInvDyn is free software: you can redistribute it
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

#include <iostream>
#include <fstream>
#include <vector>

#define PRINT_VECTOR(a) std::cout<<#a<<": "<<a.transpose()<<std::endl
#define PRINT_MATRIX(a) std::cout<<#a<<": "<<a<<std::endl

namespace pininvdyn
{
  template<typename T>
  std::string toString(const T& v)
  {
    std::stringstream ss;
    ss<<v;
    return ss.str();
  }

  template<typename T>
  std::string toString(const std::vector<T>& v, const std::string separator=", ")
  {
    std::stringstream ss;
    for(int i=0; i<v.size()-1; i++)
      ss<<v[i]<<separator;
    ss<<v[v.size()-1];
    return ss.str();
  }

  template<typename T, int n>
  std::string toString(const Eigen::MatrixBase<T>& v, const std::string separator=", ")
  {
    if(v.rows()>v.cols())
      return toString(v.transpose(), separator);
    std::stringstream ss;
    ss<<v;
    return ss.str();
  }
}

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

    void svdSolveWithDamping(ConstRefMatrix A, ConstRefVector b,
                             RefVector sol, double damping=0.0);

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

    /**
     * Write the specified matrix to a binary file with the specified name.
     */
    template<class Matrix>
    bool writeMatrixToFile(const std::string &filename, const Matrix& matrix)
    {
      std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
      if(!out.is_open())
        return false;
      typename Matrix::Index rows=matrix.rows(), cols=matrix.cols();
      out.write((char*) (&rows), sizeof(typename Matrix::Index));
      out.write((char*) (&cols), sizeof(typename Matrix::Index));
      out.write((char*) matrix.data(), rows*cols*sizeof(typename Matrix::Scalar) );
      out.close();
      return true;
    }

    /**
     * Read a matrix from the specified input binary file.
     */
    template<class Matrix>
    bool readMatrixFromFile(const std::string &filename, Matrix& matrix)
    {
      std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);
      if(!in.is_open())
        return false;
      typename Matrix::Index rows=0, cols=0;
      in.read((char*) (&rows),sizeof(typename Matrix::Index));
      in.read((char*) (&cols),sizeof(typename Matrix::Index));
      matrix.resize(rows, cols);
      in.read( (char *) matrix.data() , rows*cols*sizeof(typename Matrix::Scalar) );
      in.close();
      return true;
    }

  }
}

#endif // ifndef __invdyn_math_utils_hpp__
