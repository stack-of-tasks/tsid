//
// Copyright (c) 2018 CNRS
//

#ifndef __tsid_python_traj_sample_hpp__
#define __tsid_python_traj_sample_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include <tsid/math/utils.hpp>
#include "tsid/trajectories/trajectory-base.hpp"

#include <pinocchio/bindings/python/utils/deprecation.hpp>
#include <assert.h>
namespace tsid {
namespace python {
namespace bp = boost::python;
typedef pinocchio::SE3 SE3;

template <typename TrajSample>
struct TrajectorySamplePythonVisitor
    : public boost::python::def_visitor<
          TrajectorySamplePythonVisitor<TrajSample> > {
  template <class PyClass>

  void visit(PyClass& cl) const {
    cl.def(bp::init<unsigned int>((bp::arg("size")),
                                  "Default Constructor with size"))
        .def(bp::init<unsigned int, unsigned int>(
            (bp::arg("value_size"), bp::arg("derivative_size")),
            "Default Constructor with value and derivative size"))

        .def("resize", &TrajectorySamplePythonVisitor::resize, bp::arg("size"))
        .def("resize", &TrajectorySamplePythonVisitor::resize2,
             bp::args("value_size", "derivative_size"))

        .def("value", &TrajectorySamplePythonVisitor::value)
        .def("derivative", &TrajectorySamplePythonVisitor::derivative)
        .def("second_derivative",
             &TrajectorySamplePythonVisitor::second_derivative)

        .def("value", &TrajectorySamplePythonVisitor::setvalue_vec)
        .def("value", &TrajectorySamplePythonVisitor::setvalue_se3)
        .def("derivative", &TrajectorySamplePythonVisitor::setderivative)
        .def("second_derivative",
             &TrajectorySamplePythonVisitor::setsecond_derivative)

        // Deprecated methods:
        .def("pos", &TrajectorySamplePythonVisitor::value,
             pinocchio::python::deprecated_function<>(
                 "This method is now deprecated. Please use .value"))
        .def("vel", &TrajectorySamplePythonVisitor::derivative,
             pinocchio::python::deprecated_function<>(
                 "This method is now deprecated. Please use .derivative"))
        .def(
            "acc", &TrajectorySamplePythonVisitor::second_derivative,
            pinocchio::python::deprecated_function<>(
                "This method is now deprecated. Please use .second_derivative"))

        .def("pos", &TrajectorySamplePythonVisitor::setvalue_vec,
             pinocchio::python::deprecated_function<>(
                 "This method is now deprecated. Please use .value"))
        .def("pos", &TrajectorySamplePythonVisitor::setvalue_se3,
             pinocchio::python::deprecated_function<>(
                 "This method is now deprecated. Please use .value"))
        .def("vel", &TrajectorySamplePythonVisitor::setderivative,
             pinocchio::python::deprecated_function<>(
                 "This method is now deprecated. Please use .derivative"))
        .def("acc", &TrajectorySamplePythonVisitor::setsecond_derivative,
             pinocchio::python::deprecated_function<>(
                 "This method is now deprecated. Please use "
                 ".second_derivative"));
  }

  static void setvalue_vec(TrajSample& self, const Eigen::VectorXd value) {
    assert(self.getValue().size() == value.size());
    self.setValue(value);
  }
  static void setvalue_se3(TrajSample& self, const pinocchio::SE3& value) {
    assert(self.getValue().size() == 12);
    TSID_DISABLE_WARNING_PUSH
    TSID_DISABLE_WARNING_DEPRECATED
    tsid::math::SE3ToVector(value, self.pos);
    TSID_DISABLE_WARNING_POP
  }
  static void setderivative(TrajSample& self,
                            const Eigen::VectorXd derivative) {
    assert(self.getDerivative().size() == derivative.size());
    self.setDerivative(derivative);
  }
  static void setsecond_derivative(TrajSample& self,
                                   const Eigen::VectorXd second_derivative) {
    assert(self.getSecondDerivative().size() == second_derivative.size());
    self.setSecondDerivative(second_derivative);
  }
  static void resize(TrajSample& self, const unsigned int& size) {
    self.resize(size, size);
  }
  static void resize2(TrajSample& self, const unsigned int& value_size,
                      const unsigned int& derivative_size) {
    self.resize(value_size, derivative_size);
  }
  static Eigen::VectorXd value(const TrajSample& self) {
    return self.getValue();
  }
  static Eigen::VectorXd derivative(const TrajSample& self) {
    return self.getDerivative();
  }
  static Eigen::VectorXd second_derivative(const TrajSample& self) {
    return self.getSecondDerivative();
  }

  static void expose(const std::string& class_name) {
    std::string doc = "Trajectory Sample info.";
    bp::class_<TrajSample>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(TrajectorySamplePythonVisitor<TrajSample>());
  }
};
}  // namespace python
}  // namespace tsid

#endif  // ifndef __tsid_python_traj_euclidian_hpp__
