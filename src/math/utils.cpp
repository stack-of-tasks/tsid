//
// Copyright (c) 2017 CNRS
//

#include <tsid/math/utils.hpp>

namespace tsid {
namespace math {

void SE3ToXYZQUAT(const pinocchio::SE3& M, RefVector xyzQuat) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      xyzQuat.size() == 7, "The size of the xyzQuat vector needs to equal 7");
  xyzQuat.head<3>() = M.translation();
  xyzQuat.tail<4>() = Eigen::Quaterniond(M.rotation()).coeffs();
}

void SE3ToVector(const pinocchio::SE3& M, RefVector vec) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      vec.size() == 12, "The size of the vec vector needs to equal 12");
  vec.head<3>() = M.translation();
  typedef Eigen::Matrix<double, 9, 1> Vector9;
  vec.tail<9>() = Eigen::Map<const Vector9>(&M.rotation()(0), 9);
}

void vectorToSE3(RefVector vec, pinocchio::SE3& M) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(vec.size() == 12,
                                 "vec needs to contain 12 rows");
  M.translation(vec.head<3>());
  typedef Eigen::Matrix<double, 3, 3> Matrix3;
  M.rotation(Eigen::Map<const Matrix3>(&vec(3), 3, 3));
}

void errorInSE3(const pinocchio::SE3& M, const pinocchio::SE3& Mdes,
                pinocchio::Motion& error) {
  // error = pinocchio::log6(Mdes.inverse() * M);
  // pinocchio::SE3 M_err = Mdes.inverse() * M;
  pinocchio::SE3 M_err = M.inverse() * Mdes;
  error.linear() = M_err.translation();
  error.angular() = pinocchio::log3(M_err.rotation());
}

void solveWithDampingFromSvd(Eigen::JacobiSVD<Eigen::MatrixXd>& svd,
                             ConstRefVector b, RefVector sol, double damping) {
  assert(svd.rows() == b.size());
  const double d2 = damping * damping;
  const long int nzsv = svd.nonzeroSingularValues();
  Eigen::VectorXd tmp(svd.cols());
  tmp.noalias() = svd.matrixU().leftCols(nzsv).adjoint() * b;
  double sv;
  for (long int i = 0; i < nzsv; i++) {
    sv = svd.singularValues()(i);
    tmp(i) *= sv / (sv * sv + d2);
  }
  sol = svd.matrixV().leftCols(nzsv) * tmp;
  //  cout<<"sing val = "+toString(svd.singularValues(),3);
  //  cout<<"solution with damp "+toString(damping)+" = "+toString(res.norm());
  //  cout<<"solution without damping  ="+toString(svd.solve(b).norm());
}

void svdSolveWithDamping(ConstRefMatrix A, ConstRefVector b, RefVector sol,
                         double damping) {
  assert(A.rows() == b.size());
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(A.rows(), A.cols());
  svd.compute(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  solveWithDampingFromSvd(svd, b, sol, damping);
}

void pseudoInverse(ConstRefMatrix A, RefMatrix Apinv, double tolerance,
                   unsigned int computationOptions)

{
  Eigen::JacobiSVD<Eigen::MatrixXd> svdDecomposition(A.rows(), A.cols());
  pseudoInverse(A, svdDecomposition, Apinv, tolerance, computationOptions);
}

void pseudoInverse(ConstRefMatrix A,
                   Eigen::JacobiSVD<Eigen::MatrixXd>& svdDecomposition,
                   RefMatrix Apinv, double tolerance,
                   unsigned int computationOptions) {
  using namespace Eigen;
  int nullSpaceRows = -1, nullSpaceCols = -1;
  pseudoInverse(A, svdDecomposition, Apinv, tolerance, (double*)0,
                nullSpaceRows, nullSpaceCols, computationOptions);
}

void pseudoInverse(ConstRefMatrix A,
                   Eigen::JacobiSVD<Eigen::MatrixXd>& svdDecomposition,
                   RefMatrix Apinv, double tolerance, double* nullSpaceBasisOfA,
                   int& nullSpaceRows, int& nullSpaceCols,
                   unsigned int computationOptions) {
  using namespace Eigen;

  if (computationOptions == 0)
    return;  // if no computation options we cannot compute the pseudo inverse
  svdDecomposition.compute(A, computationOptions);

  JacobiSVD<MatrixXd>::SingularValuesType singularValues =
      svdDecomposition.singularValues();
  long int singularValuesSize = singularValues.size();
  int rank = 0;
  for (long int idx = 0; idx < singularValuesSize; idx++) {
    if (tolerance > 0 && singularValues(idx) > tolerance) {
      singularValues(idx) = 1.0 / singularValues(idx);
      rank++;
    } else {
      singularValues(idx) = 0.0;
    }
  }

  // equivalent to this U/V matrix in case they are computed full
  Apinv = svdDecomposition.matrixV().leftCols(singularValuesSize) *
          singularValues.asDiagonal() *
          svdDecomposition.matrixU().leftCols(singularValuesSize).adjoint();

  if (nullSpaceBasisOfA && (computationOptions & ComputeFullV)) {
    // we can compute the null space basis for A
    nullSpaceBasisFromDecomposition(svdDecomposition, rank, nullSpaceBasisOfA,
                                    nullSpaceRows, nullSpaceCols);
  }
}

void dampedPseudoInverse(ConstRefMatrix A,
                         Eigen::JacobiSVD<Eigen::MatrixXd>& svdDecomposition,
                         RefMatrix Apinv, double tolerance,
                         double dampingFactor, unsigned int computationOptions,
                         double* nullSpaceBasisOfA, int* nullSpaceRows,
                         int* nullSpaceCols) {
  using namespace Eigen;

  if (computationOptions == 0)
    return;  // if no computation options we cannot compute the pseudo inverse
  svdDecomposition.compute(A, computationOptions);

  JacobiSVD<MatrixXd>::SingularValuesType singularValues =
      svdDecomposition.singularValues();

  // rank will be used for the null space basis.
  // not sure if this is correct
  const long int singularValuesSize = singularValues.size();
  const double d2 = dampingFactor * dampingFactor;
  int rank = 0;
  for (int idx = 0; idx < singularValuesSize; idx++) {
    if (singularValues(idx) > tolerance) rank++;
    singularValues(idx) = singularValues(idx) /
                          ((singularValues(idx) * singularValues(idx)) + d2);
  }

  // equivalent to this U/V matrix in case they are computed full
  Apinv = svdDecomposition.matrixV().leftCols(singularValuesSize) *
          singularValues.asDiagonal() *
          svdDecomposition.matrixU().leftCols(singularValuesSize).adjoint();

  if (nullSpaceBasisOfA && nullSpaceRows && nullSpaceCols &&
      (computationOptions & ComputeFullV)) {
    // we can compute the null space basis for A
    nullSpaceBasisFromDecomposition(svdDecomposition, rank, nullSpaceBasisOfA,
                                    *nullSpaceRows, *nullSpaceCols);
  }
}

void nullSpaceBasisFromDecomposition(
    const Eigen::JacobiSVD<Eigen::MatrixXd>& svdDecomposition, double tolerance,
    double* nullSpaceBasisMatrix, int& rows, int& cols) {
  using namespace Eigen;
  JacobiSVD<MatrixXd>::SingularValuesType singularValues =
      svdDecomposition.singularValues();
  int rank = 0;
  for (int idx = 0; idx < singularValues.size(); idx++) {
    if (tolerance > 0 && singularValues(idx) > tolerance) {
      rank++;
    }
  }
  nullSpaceBasisFromDecomposition(svdDecomposition, rank, nullSpaceBasisMatrix,
                                  rows, cols);
}

void nullSpaceBasisFromDecomposition(
    const Eigen::JacobiSVD<Eigen::MatrixXd>& svdDecomposition, int rank,
    double* nullSpaceBasisMatrix, int& rows, int& cols) {
  using namespace Eigen;
  const MatrixXd& vMatrix = svdDecomposition.matrixV();
  // A \in \mathbb{R}^{uMatrix.rows() \times vMatrix.cols()}
  rows = (int)vMatrix.cols();
  cols = (int)vMatrix.cols() - rank;
  Map<MatrixXd> map(nullSpaceBasisMatrix, rows, cols);
  map = vMatrix.rightCols(vMatrix.cols() - rank);
}

}  // namespace math
}  // namespace tsid
