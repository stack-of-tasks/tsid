//
// Copyright (c) 2022 INRIA
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

#include "tsid/bindings/python/fwd.hpp"
#include <pinocchio/bindings/python/utils/deprecation.hpp>
#include "tsid/bindings/python/math/utils.hpp"
#include "tsid/math/utils.hpp"

namespace tsid
{
  namespace python
  {
    void exposeMathUtils()
    {
        namespace bp = boost::python;
        using namespace math;

        bp::def("SE3ToVector", &SE3ToVector, bp::args("M", "vec"), "Convert the input SE3 object M to a 12D vector of floats [X,Y,Z,R11,R12,R13,R14,...] vec");
        bp::def("vectorToSE3", &vectorToSE3, bp::args("vec", "M"), "Convert the input 12D vector of floats [X,Y,Z,R11,R12,R13,R14,...] vec to a SE3 object M");
    }
  }
}
