//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// PinInvDyn is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// PinInvDyn is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// PinInvDyn If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef _invdyn_lib_config_hpp
#define _invdyn_lib_config_hpp

// Package version (header).
# define PININVDYN_VERSION "UNKNOWN"

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
#  define PININVDYN_DLLIMPORT __declspec(dllimport)
#  define PININVDYN_DLLEXPORT __declspec(dllexport)
#  define PININVDYN_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define PININVDYN_DLLIMPORT __attribute__ ((visibility("default")))
#   define PININVDYN_DLLEXPORT __attribute__ ((visibility("default")))
#   define PININVDYN_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define PININVDYN_DLLIMPORT
#   define PININVDYN_DLLEXPORT
#   define PININVDYN_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef PININVDYN_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define PININVDYN_DLLAPI
#  define PININVDYN_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef PININVDYN_EXPORTS
#   define PININVDYN_DLLAPI PININVDYN_DLLEXPORT
#  else
#   define PININVDYN_DLLAPI PININVDYN_DLLIMPORT
#  endif // PININVDYN_EXPORTS
#  define PININVDYN_LOCAL PININVDYN_DLLLOCAL
# endif // PININVDYN_STATIC

#endif //_invdyn_lib_config_hpp
