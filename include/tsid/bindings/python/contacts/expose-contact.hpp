//
// Copyright (c) 2018 CNRS, NYU, MPI Tübingen
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

#ifndef __tsid_python_expose_contact_hpp__
#define __tsid_python_expose_contact_hpp__

#include "tsid/bindings/python/contacts/contact-6d.hpp"
#include "tsid/bindings/python/contacts/contact-point.hpp"
#include "tsid/bindings/python/contacts/contact-two-frame-positions.hpp"
#include "tsid/bindings/python/contacts/measured-6d-wrench.hpp"

namespace tsid {
namespace python {
void exposeContact6d();
void exposeContactPoint();
void exposeContactTwoFramePositions();
void exposeMeasured6dWrench();

inline void exposeContact() {
  exposeContact6d();
  exposeContactPoint();
  exposeContactTwoFramePositions();
  exposeMeasured6dWrench();
}

}  // namespace python
}  // namespace tsid
#endif  // ifndef __tsid_python_expose_contact_hpp__