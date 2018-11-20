#ifndef __tsid_python_constriant_equality_hpp__
#define __tsid_python_constriant_equality_hpp__

#include <boost/python.hpp>
#include <string>
#include <eigenpy/eigenpy.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

#include "tsid/math/constraint-equality.hpp"

namespace tsid
{
  namespace python
  {    
    namespace bp = boost::python;
    
    template<typename ConstraintEquality>
    struct ConstraintEqPythonVisitor
    : public boost::python::def_visitor< ConstraintEqPythonVisitor<ConstraintEquality> >
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      typedef double Scalar;
      typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> Vector;
      typedef Eigen::Ref<Vector>              RefVector;
      typedef const Eigen::Ref<const Vector>  ConstRefVector;

      template<class PyClass>     

      void visit(PyClass& cl) const
      {
        cl
        .def(bp::init<std::string>((bp::arg("name")), "Default constructor with name."))
        .def(bp::init<std::string, unsigned int, unsigned int>((bp::arg("name"), bp::arg("row"), bp::arg("col")), "Default constructor with name and size."))
        .def(bp::init<std::string, Eigen::MatrixXd, Eigen::VectorXd>((bp::arg("name"), bp::arg("A"), bp::arg("b")), "Default constructor with name and constraint."))
        
        .add_property("rows", &ConstraintEquality::rows)
        .add_property("cols", &ConstraintEquality::cols)
        .def("resize",&ConstraintEquality::resize, (bp::arg("r"), bp::arg("c")),"Resize constraint size.")
        
        .add_property("isEquality", &ConstraintEquality::isEquality)
        .add_property("isInequality", &ConstraintEquality::isInequality)
        .add_property("isBound", &ConstraintEquality::isBound)
        
        .add_property("matrix", &ConstraintEqPythonVisitor::matrix)
        .add_property("vector", &ConstraintEqPythonVisitor::vector)
        .add_property("lowerBound", &ConstraintEqPythonVisitor::lowerBound)
        .add_property("upperBound", &ConstraintEqPythonVisitor::upperBound)

        .def("setMatrix", (bool (ConstraintEquality::*)(const Eigen::MatrixXd &) const) &ConstraintEquality::setMatrix, bp::args("matrix"), "Set Matrix")
        .def("setVector", (bool (ConstraintEquality::*)(const Eigen::VectorXd &) const) &ConstraintEquality::setVector, bp::args("vector"), "Set Vector")
        .def("setLowerBound", (bool (ConstraintEquality::*)(const Eigen::VectorXd &) const) &ConstraintEquality::setLowerBound, bp::args("lb"), "Set LowerBound")
        .def("setUpperBound", (bool (ConstraintEquality::*)(const Eigen::VectorXd &) const) &ConstraintEquality::setUpperBound, bp::args("ub"), "Set UpperBound")
        
        ;
      }
      static Eigen::MatrixXd matrix (const ConstraintEquality & self) {return self.matrix();}
      static Eigen::VectorXd vector (const ConstraintEquality & self) {return self.vector();}
      static Eigen::VectorXd lowerBound (const ConstraintEquality & self) {return self.lowerBound();}
      static Eigen::VectorXd upperBound (const ConstraintEquality & self) {return self.upperBound();}
      
      static void expose(const std::string & class_name)
      {
        std::string doc = "Constraint Equality info.";
        bp::class_<ConstraintEquality>(class_name.c_str(),
                          doc.c_str(),
                          bp::no_init)
        .def(ConstraintEqPythonVisitor<ConstraintEquality>());
        
        eigenpy::enableEigenPySpecific<Eigen::MatrixXd>();
      }
    };
  }
}


#endif // ifndef __tsid_python_constriant_equality_hpp__