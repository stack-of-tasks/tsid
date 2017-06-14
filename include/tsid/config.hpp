//
// Copyright (c) 2017 CNRS
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

#ifndef _invdyn_lib_config_hpp
#define _invdyn_lib_config_hpp

// Package version (header).
# define TSID_VERSION "UNKNOWN"

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
# if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define TSID_DLLIMPORT __declspec(dllimport)
#  define TSID_DLLEXPORT __declspec(dllexport)
#  define TSID_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define TSID_DLLIMPORT __attribute__ ((visibility("default")))
#   define TSID_DLLEXPORT __attribute__ ((visibility("default")))
#   define TSID_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define TSID_DLLIMPORT
#   define TSID_DLLEXPORT
#   define TSID_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef TSID_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define TSID_DLLAPI
#  define TSID_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef TSID_EXPORTS
#   define TSID_DLLAPI TSID_DLLEXPORT
#  else
#   define TSID_DLLAPI TSID_DLLIMPORT
#  endif // TSID_EXPORTS
#  define TSID_LOCAL TSID_DLLLOCAL
# endif // TSID_STATIC

#endif //_invdyn_lib_config_hpp
