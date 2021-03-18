//
// Copyright (c) 2021 CNRS - INRIA
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

#ifndef __tsid_python_fwd_hpp__
#define __tsid_python_fwd_hpp__

#include <string>

// Silence a warning about a deprecated use of boost bind by boost python
// at least fo boost 1.73 to 1.75
// ref. https://github.com/stack-of-tasks/tsid/issues/128
#define BOOST_BIND_GLOBAL_PLACEHOLDERS

#include <pinocchio/fwd.hpp>
#include <eigenpy/eigenpy.hpp>

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/make_constructor.hpp>

#undef BOOST_BIND_GLOBAL_PLACEHOLDERS

#endif
