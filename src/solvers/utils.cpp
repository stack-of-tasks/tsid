//
// Copyright (c) 2017 CNRS
//

#include "tsid/solvers/utils.hpp"
#include "tsid/math/constraint-base.hpp"

#include <iostream>

namespace tsid {
namespace solvers {

std::string HQPDataToString(const HQPData& data, bool printMatrices) {
  using namespace std;

  stringstream ss;
  unsigned int priority = 0;
  for (HQPData::const_iterator it = data.begin(); it != data.end(); it++) {
    ss << "Level " << priority << endl;
    for (ConstraintLevel::const_iterator iit = it->begin(); iit != it->end();
         iit++) {
      auto c = iit->second;
      ss << " - " << c->name() << ": w=" << iit->first << ", ";
      if (c->isEquality())
        ss << "equality, ";
      else if (c->isInequality())
        ss << "inequality, ";
      else
        ss << "bound, ";
      ss << c->rows() << "x" << c->cols() << endl;
    }
    priority++;
  }

  if (printMatrices) {
    ss << endl;
    for (HQPData::const_iterator it = data.begin(); it != data.end(); it++) {
      for (ConstraintLevel::const_iterator iit = it->begin(); iit != it->end();
           iit++) {
        auto c = iit->second;
        ss << "*** " << c->name() << " *** ";
        if (c->isEquality()) {
          ss << "(equality)" << endl;
          ss << "A =\n" << c->matrix() << endl;
          ss << "b = " << c->vector().transpose() << endl;
        } else if (c->isInequality()) {
          ss << "(inequality)" << endl;
          ss << "A =\n" << c->matrix() << endl;
          ss << "lb = " << c->lowerBound().transpose() << endl;
          ss << "ub = " << c->upperBound().transpose() << endl;
        } else {
          ss << "(bounds)" << endl;
          ss << "lb = " << c->lowerBound().transpose() << endl;
          ss << "ub = " << c->upperBound().transpose() << endl;
        }
        ss << endl;
      }
    }
  }
  return ss.str();
}

}  // namespace solvers
}  // namespace tsid
