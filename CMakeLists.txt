#
# Copyright (c) 2017 CNRS
#
# This file is part of PinInvDyn
# pinocchio is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
# pinocchio is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# pinocchio If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake2/ide.cmake)

SET(PROJECT_NAME PinInvDyn)
SET(PROJECT_DESCRIPTION "Efficient Inverse Dynamics for Multi-body Systems")
SET(PROJECT_URL "http://github.com/stack-of-tasks/pinocchio")
OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" FALSE)

SET(DOXYGEN_USE_MATHJAX YES)

IF(APPLE)
  SET(CMAKE_MACOSX_RPATH TRUE)
  SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
     set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  endif("${isSystemDir}" STREQUAL "-1")
ENDIF(APPLE)

# Disable -Werror on Unix for now.
SET(CXX_DISABLE_WERROR True)
SET(CMAKE_VERBOSE_MAKEFILE True)

SETUP_PROJECT()

IF(WIN32)
  SET(LINK copy_if_different)
ELSE(WIN32)
  SET(LINK create_symlink)
ENDIF(WIN32)

# --- OPTIONS ----------------------------------------
OPTION (BUILD_PYTHON_INTERFACE "Build the python binding" OFF)
OPTION (BUILD_UNIT_TESTS "Build the unitary tests" ON)

# ----------------------------------------------------
# --- DEPENDANCIES -----------------------------------
# ----------------------------------------------------
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.5")
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2.0")

SET(BOOST_COMPONENTS unit_test_framework serialization)

IF(BUILD_PYTHON_INTERFACE)
  ADD_COMPILE_DEPENDENCY("eigenpy >= 1.3.1")
  SET(BOOST_COMPONENTS ${BOOST_COMPONENTS} python)
  FINDPYTHON(2.7 EXACT REQUIRED)
  INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_DIRS}) 
ENDIF(BUILD_PYTHON_INTERFACE)

SEARCH_FOR_BOOST()
# Path to boost headers
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------

SET(${PROJECT_NAME}_MATH_HEADERS
    include/pininvdyn/math/fwd.hpp
  )

SET(${PROJECT_NAME}_TASKS_HEADERS
    include/pininvdyn/tasks/fwd.hpp
    include/pininvdyn/tasks/task-base.hpp
  )

SET(${PROJECT_NAME}_SOLVERS_HEADERS
    include/pininvdyn/solvers/fwd.hpp
  )

SET(HEADERS
    ${${PROJECT_NAME}_MATH_HEADERS}
    ${${PROJECT_NAME}_TASKS_HEADERS}
    ${${PROJECT_NAME}_SOLVERS_HEADERS}
 )

LIST(REMOVE_DUPLICATES HEADERS)

SET(HEADERS_FULL_PATH "")
FOREACH(header ${HEADERS})
  LIST(APPEND HEADERS_FULL_PATH "${CMAKE_SOURCE_DIR}/${header}") 
  GET_FILENAME_COMPONENT(headerName ${header} NAME)
  GET_FILENAME_COMPONENT(headerPath ${header} PATH)
  INSTALL(FILES ${${PROJECT_NAME}_SOURCE_DIR}/${header}
    DESTINATION ${CMAKE_INSTALL_PREFIX}/${headerPath}
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
ENDFOREACH(header ${HEADERS})

# --- MAIN LIBRARY -------------------------------------------------------------
ADD_SUBDIRECTORY(src)

# --- BINDINGS ----------------------------------------------------------------
#ADD_SUBDIRECTORY(bindings)

# --- UNIT TESTS ---------------------------------------------------------------
#ADD_SUBDIRECTORY(unittest)

# --- PACKAGING ----------------------------------------------------------------
PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
PKG_CONFIG_APPEND_BOOST_LIBS(serialization)

SETUP_PROJECT_FINALIZE()
