//
// Copyright (c) 2018 CNRS
//

#ifndef __tsid_python_traj_euclidian_hpp__
#define __tsid_python_traj_euclidian_hpp__

#include "tsid/bindings/python/fwd.hpp"

#include "tsid/trajectories/trajectory-euclidian.hpp"
namespace tsid {
namespace python {
namespace bp = boost::python;

template <typename Traj>
struct TrajectoryEuclidianConstantPythonVisitor
    : public boost::python::def_visitor<
          TrajectoryEuclidianConstantPythonVisitor<Traj> > {
  template <class PyClass>

  void visit(PyClass& cl) const {
    cl.def(bp::init<std::string>((bp::arg("name")),
                                 "Default Constructor with name"))
        .def(bp::init<std::string, Eigen::VectorXd>(
            (bp::arg("name"), bp::arg("reference")),
            "Default Constructor with name and ref_vec"))

        .add_property("size", &Traj::size)
        .def("setReference",
             &TrajectoryEuclidianConstantPythonVisitor::setReference,
             bp::arg("ref_vec"))
        .def("computeNext",
             &TrajectoryEuclidianConstantPythonVisitor::computeNext)
        .def("getLastSample",
             &TrajectoryEuclidianConstantPythonVisitor::getLastSample,
             bp::arg("sample"))
        .def("has_trajectory_ended",
             &TrajectoryEuclidianConstantPythonVisitor::has_trajectory_ended)
        .def("getSample", &TrajectoryEuclidianConstantPythonVisitor::getSample,
             bp::arg("time"));
  }
  static void setReference(Traj& self, const Eigen::VectorXd& ref) {
    self.setReference(ref);
  }
  static trajectories::TrajectorySample computeNext(Traj& self) {
    return self.computeNext();
  }
  static void getLastSample(const Traj& self,
                            trajectories::TrajectorySample& sample) {
    self.getLastSample(sample);
  }
  static bool has_trajectory_ended(const Traj& self) {
    return self.has_trajectory_ended();
  }
  static trajectories::TrajectorySample getSample(Traj& self, double time) {
    return self.operator()(time);
  }

  static void expose(const std::string& class_name) {
    std::string doc = "Trajectory Euclidian Constant info.";
    bp::class_<Traj>(class_name.c_str(), doc.c_str(), bp::no_init)
        .def(TrajectoryEuclidianConstantPythonVisitor<Traj>());
  }
};
}  // namespace python
}  // namespace tsid

#endif  // ifndef __tsid_python_traj_euclidian_hpp__
