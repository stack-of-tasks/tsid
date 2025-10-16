//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/fwd.hpp"
#include <eigenpy/geometry.hpp>

#include "tsid/bindings/python/robots/expose-robots.hpp"
#include "tsid/bindings/python/constraint/expose-constraints.hpp"
#include "tsid/bindings/python/contacts/expose-contact.hpp"
#include "tsid/bindings/python/trajectories/expose-trajectories.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"
#include "tsid/bindings/python/solvers/expose-solvers.hpp"
#include "tsid/bindings/python/formulations/expose-formulations.hpp"
#include "tsid/bindings/python/math/utils.hpp"

namespace bp = boost::python;
using namespace tsid::python;

BOOST_PYTHON_MODULE(tsid_pywrap) {
  eigenpy::enableEigenPy();
  eigenpy::exposeAngleAxis();
  eigenpy::exposeQuaternion();

  typedef Eigen::Matrix<double, 6, 6> Matrix6d;
  typedef Eigen::Matrix<double, 6, 1> Vector6d;
  typedef Eigen::Matrix<double, 6, Eigen::Dynamic> Matrix6x;
  typedef Eigen::Matrix<double, 3, Eigen::Dynamic> Matrix3x;

  eigenpy::enableEigenPySpecific<Matrix6d>();
  eigenpy::enableEigenPySpecific<Vector6d>();
  eigenpy::enableEigenPySpecific<Matrix6x>();
  eigenpy::enableEigenPySpecific<Matrix3x>();
  eigenpy::enableEigenPySpecific<Eigen::MatrixXd>();
  eigenpy::enableEigenPySpecific<Eigen::Vector3d>();

  exposeRobots();
  exposeConstraints();
  exposeContact();
  exposeTrajectories();
  exposeTasks();
  exposeSolvers();
  exposeFormulations();
  exposeMathUtils();
}
