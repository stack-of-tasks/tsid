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

#ifndef __invdyn_math_utils_hpp__
#define __invdyn_math_utils_hpp__

#include "tsid/math/fwd.hpp"

#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <iostream>
#include <fstream>
#include <vector>

#define PRINT_VECTOR(a) std::cout<<#a<<"("<<a.rows()<<"x"<<a.cols()<<"): "<<a.transpose().format(math::CleanFmt)<<std::endl
#define PRINT_MATRIX(a) std::cout<<#a<<"("<<a.rows()<<"x"<<a.cols()<<"):\n"<<a.format(math::CleanFmt)<<std::endl

namespace tsid
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

namespace tsid
{
  namespace math
  {
    static const Eigen::IOFormat CleanFmt(1, 0, ", ", "\n", "[", "]");

    /** List of available parameters of IOFormat constructor:
        precision       number of digits for floating point values, or one of the special constants StreamPrecision and FullPrecision.
        flags           either 0, or DontAlignCols, which allows to disable the alignment of columns, resulting in faster code.
        coeffSeparator  string printed between two coefficients of the same row
        rowSeparator    string printed between two rows
        rowPrefix       string printed at the beginning of each row
        rowSuffix       string printed at the end of each row
        matPrefix       string printed at the beginning of the matrix
        matSuffix       string printed at the end of the matrix */
    static const Eigen::IOFormat matlabPrintFormat(Eigen::FullPrecision, Eigen::DontAlignCols, " ", ";\n", "", "", "[", "];");

    /**
     * Convert the input SE3 object to a 7D vector of floats [X,Y,Z,Q1,Q2,Q3,Q4].
     */
    void SE3ToXYZQUAT(const pinocchio::SE3 & M, RefVector xyzQuat);

    /**
     * Convert the input SE3 object to a 12D vector of floats [X,Y,Z,R11,R12,R13,R14,...].
     */
    void SE3ToVector(const pinocchio::SE3 & M, RefVector vec);

    void vectorToSE3(RefVector vec, pinocchio::SE3 & M);

    void errorInSE3 (const pinocchio::SE3 & M,
                     const pinocchio::SE3 & Mdes,
                     pinocchio::Motion & error);

    void solveWithDampingFromSvd(Eigen::JacobiSVD<Eigen::MatrixXd> & svd,
                             ConstRefVector b,
                             RefVector sol, double damping=0.0);

    void svdSolveWithDamping(ConstRefMatrix A, ConstRefVector b,
                             RefVector sol, double damping=0.0);

    void pseudoInverse(ConstRefMatrix A,
                       RefMatrix Apinv,
                       double tolerance,
                       unsigned int computationOptions = Eigen::ComputeThinU | Eigen::ComputeThinV);

    void pseudoInverse(ConstRefMatrix A,
                       Eigen::JacobiSVD<Eigen::MatrixXd>& svdDecomposition,
                       RefMatrix Apinv,
                       double tolerance,
                       unsigned int computationOptions);

    void pseudoInverse(ConstRefMatrix A,
                       Eigen::JacobiSVD<Eigen::MatrixXd>& svdDecomposition,
                       RefMatrix Apinv,
                       double tolerance,
                       double * nullSpaceBasisOfA,
                       int &nullSpaceRows,
                       int &nullSpaceCols,
                       unsigned int computationOptions);

    void dampedPseudoInverse(ConstRefMatrix A,
                             Eigen::JacobiSVD<Eigen::MatrixXd>& svdDecomposition,
                             RefMatrix Apinv,
                             double tolerance,
                             double dampingFactor,
                             unsigned int computationOptions = Eigen::ComputeThinU | Eigen::ComputeThinV,
                             double * nullSpaceBasisOfA=0,
                             int *nullSpaceRows=0, int *nullSpaceCols=0);

    void nullSpaceBasisFromDecomposition(const Eigen::JacobiSVD<Eigen::MatrixXd> & svdDecomposition,
                                         double tolerance,
                                         double * nullSpaceBasisMatrix,
                                         int &rows, int &cols);
    
    void nullSpaceBasisFromDecomposition(const Eigen::JacobiSVD<Eigen::MatrixXd> & svdDecomposition,
                                         int rank,
                                         double * nullSpaceBasisMatrix,
                                         int &rows, int &cols);

    template<typename Derived>
    inline bool isFinite(const Eigen::MatrixBase<Derived>& x)
    {
      return ( (x - x).array() == (x - x).array()).all();
    }

    template<typename Derived>
    inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
    {
            return ((x.array() == x.array())).all();
    }

    /**
     * Write the specified matrix to a binary file with the specified name.
     */
    template<class Matrix>
    bool writeMatrixToFile(const std::string & filename,
                           const Eigen::MatrixBase<Matrix> & matrix)
    {
      typedef typename Matrix::Index Index;
      typedef typename Matrix::Scalar Scalar;
      
      std::ofstream out(filename.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
      if(!out.is_open())
        return false;
      Index rows=matrix.rows(), cols=matrix.cols();
      out.write((char*) (&rows), sizeof(Index));
      out.write((char*) (&cols), sizeof(Index));
      out.write((char*) matrix.data(), rows*cols*sizeof(Scalar) );
      out.close();
      return true;
    }

    /**
     * Read a matrix from the specified input binary file.
     */
    template<class Matrix>
    bool readMatrixFromFile(const std::string & filename,
                            const Eigen::MatrixBase<Matrix> & matrix)
    {
      typedef typename Matrix::Index Index;
      typedef typename Matrix::Scalar Scalar;
      
      std::ifstream in(filename.c_str(), std::ios::in | std::ios::binary);
      if(!in.is_open())
        return false;
      Index rows=0, cols=0;
      in.read((char*) (&rows),sizeof(Index));
      in.read((char*) (&cols),sizeof(Index));
      
      Eigen::MatrixBase<Matrix> & matrix_ = const_cast< Eigen::MatrixBase<Matrix>& >(matrix);
      
      matrix_.resize(rows, cols);
      in.read( (char *) matrix_.data() , rows*cols*sizeof(Scalar) );
      in.close();
      return true;
    }

  }
}

#endif // ifndef __invdyn_math_utils_hpp__
