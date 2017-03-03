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

#include <pininvdyn/math/utils.hpp>

void pininvdyn::math::se3ToXYZQUAT(const se3::SE3 & M, RefVector xyzQuat)
{
  assert(xyzQuat.size()>=7);
  xyzQuat.head<3>() = M.translation();
  xyzQuat.tail<4>() = Eigen::Quaterniond(M.rotation()).coeffs();
}

void pininvdyn::math::se3ToVector(const se3::SE3 & M, RefVector vec)
{
  assert(vec.size()>=12);
  vec.head<3>() = M.translation();
  typedef Eigen::Matrix<double,9,1> Vector9;
  vec.tail<9>() = Eigen::Map<const Vector9>(&M.rotation()(0), 9);
}

void pininvdyn::math::vectorToSE3(RefVector vec, se3::SE3 & M)
{
  assert(vec.size()>=12);
  M.translation( vec.head<3>() );
  typedef Eigen::Matrix<double,3,3> Matrix3;
  M.rotation( Eigen::Map<const Matrix3>(&vec(3), 3, 3) );
}

void pininvdyn::math::errorInSE3 (const se3::SE3 & M,
                                  const se3::SE3 & Mdes,
                                  se3::Motion & error)
{
  error = se3::log6(Mdes.inverse() * M);
}


void pininvdyn::math::pseudoInverse(ConstRefMatrix A,
                                    RefMatrix Apinv,
                                    double tolerance,
                                    unsigned int computationOptions)

{
  Eigen::JacobiSVD<typename Eigen::MatrixXd::PlainObject> svdDecomposition(A.rows(), A.cols());
  pininvdyn::math::pseudoInverse(A, svdDecomposition, Apinv, tolerance, computationOptions);
}

void pininvdyn::math::pseudoInverse(ConstRefMatrix A,
                                    Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                                    RefMatrix Apinv,
                                    double tolerance,
                                    unsigned int computationOptions)
{
  using namespace Eigen;
  int nullSpaceRows = -1, nullSpaceCols = -1;
  pseudoInverse(A, svdDecomposition, Apinv, tolerance,
                (double*)0, nullSpaceRows, nullSpaceCols, computationOptions);
}

void pininvdyn::math::pseudoInverse(ConstRefMatrix A,
                                    Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                                    RefMatrix Apinv,
                                    double tolerance,
                                    double * nullSpaceBasisOfA,
                                    int &nullSpaceRows, int &nullSpaceCols,
                                    unsigned int computationOptions)
{
  using namespace Eigen;

  if (computationOptions == 0) return; //if no computation options we cannot compute the pseudo inverse
  svdDecomposition.compute(A, computationOptions);

  JacobiSVD<MatrixXd::PlainObject>::SingularValuesType singularValues = svdDecomposition.singularValues();
  int singularValuesSize = singularValues.size();
  int rank = 0;
  for (int idx = 0; idx < singularValuesSize; idx++) {
    if (tolerance > 0 && singularValues(idx) > tolerance) {
      singularValues(idx) = 1.0 / singularValues(idx);
      rank++;
    } else {
      singularValues(idx) = 0.0;
    }
  }

  //equivalent to this U/V matrix in case they are computed full
  Apinv = svdDecomposition.matrixV().leftCols(singularValuesSize) * singularValues.asDiagonal() * svdDecomposition.matrixU().leftCols(singularValuesSize).adjoint();

  if (nullSpaceBasisOfA && (computationOptions & ComputeFullV)) {
    //we can compute the null space basis for A
    nullSpaceBasisFromDecomposition(svdDecomposition, rank, nullSpaceBasisOfA, nullSpaceRows, nullSpaceCols);
  }
}

void pininvdyn::math::dampedPseudoInverse(ConstRefMatrix A,
                                          Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                                          RefMatrix Apinv,
                                          double tolerance,
                                          double dampingFactor,
                                          unsigned int computationOptions,
                                          double * nullSpaceBasisOfA,
                                          int *nullSpaceRows, int *nullSpaceCols)
{
  using namespace Eigen;

  if (computationOptions == 0) return; //if no computation options we cannot compute the pseudo inverse
  svdDecomposition.compute(A, computationOptions);

  JacobiSVD<MatrixXd::PlainObject>::SingularValuesType singularValues = svdDecomposition.singularValues();

  //rank will be used for the null space basis.
  //not sure if this is correct
  const int singularValuesSize = singularValues.size();
  const double d2 = dampingFactor * dampingFactor;
  int rank = 0;
  for (int idx = 0; idx < singularValuesSize; idx++)
  {
    if(singularValues(idx) > tolerance)
      rank++;
    singularValues(idx) = singularValues(idx) / ((singularValues(idx) * singularValues(idx)) + d2);
  }

  //equivalent to this U/V matrix in case they are computed full
  Apinv = svdDecomposition.matrixV().leftCols(singularValuesSize) * singularValues.asDiagonal() * svdDecomposition.matrixU().leftCols(singularValuesSize).adjoint();

  if (nullSpaceBasisOfA && nullSpaceRows && nullSpaceCols
      && (computationOptions & ComputeFullV))
  {
    //we can compute the null space basis for A
    nullSpaceBasisFromDecomposition(svdDecomposition, rank, nullSpaceBasisOfA, *nullSpaceRows, *nullSpaceCols);
  }
}

void pininvdyn::math::nullSpaceBasisFromDecomposition(Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                                                      double tolerance,
                                                      double * nullSpaceBasisMatrix,
                                                      int &rows, int &cols)
{
  using namespace Eigen;
  JacobiSVD<MatrixXd::PlainObject>::SingularValuesType singularValues = svdDecomposition.singularValues();
  int rank = 0;
  for (int idx = 0; idx < singularValues.size(); idx++) {
    if (tolerance > 0 && singularValues(idx) > tolerance) {
      rank++;
    }
  }
  nullSpaceBasisFromDecomposition(svdDecomposition, rank, nullSpaceBasisMatrix, rows, cols);

}

void nullSpaceBasisFromDecomposition(Eigen::JacobiSVD<Eigen::MatrixXd::PlainObject>& svdDecomposition,
                                     int rank,
                                     double * nullSpaceBasisMatrix,
                                     int &rows, int &cols)
{
  using namespace Eigen;
  const MatrixXd &vMatrix = svdDecomposition.matrixV();
  //A \in \mathbb{R}^{uMatrix.rows() \times vMatrix.cols()}
  rows = vMatrix.cols();
  cols = vMatrix.cols() - rank;
  Map<MatrixXd> map(nullSpaceBasisMatrix, rows, cols);
  map = vMatrix.rightCols(vMatrix.cols() - rank);
}
