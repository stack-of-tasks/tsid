//
// Copyright (c) 2018 CNRS
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

#ifndef __tsid_python_expose_solvers_hpp__
#define __tsid_python_expose_solvers_hpp__


#include "tsid/bindings/python/solvers/solver-HQP-eiquadprog.hpp"
#include "tsid/bindings/python/solvers/HQPData.hpp"
#include "tsid/bindings/python/solvers/HQPOutput.hpp"
namespace tsid
{
  namespace python
  {
    void exposeSolverHQuadProg();
    void exposeConstraintLevel();
    void exposeHQPData();
    void exposeHQPOutput();
    inline void exposeSolvers()
    {
      exposeSolverHQuadProg();
      exposeConstraintLevel();
      exposeHQPData();
      exposeHQPOutput();
    }
    
  } // namespace python
} // namespace tsid
#endif // ifndef __tsid_python_expose_solvers_hpp__
