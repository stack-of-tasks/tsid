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
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
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
OPTION (INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" OFF)
OPTION (EIGEN_RUNTIME_NO_MALLOC "If ON, it can assert in case of runtime allocation" ON)
OPTION (EIGEN_NO_AUTOMATIC_RESIZING "If ON, it forbids automatic resizing of dynamics arrays and matrices" OFF)

IF(INITIALIZE_WITH_NAN)
  MESSAGE(STATUS "Initialize with NaN all the Eigen entries.")
  ADD_DEFINITIONS(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
ENDIF(INITIALIZE_WITH_NAN)

IF(EIGEN_RUNTIME_NO_MALLOC)
  MESSAGE(STATUS "Option EIGEN_RUNTIME_NO_MALLOC on.")
  ADD_DEFINITIONS(-DEIGEN_RUNTIME_NO_MALLOC)
ENDIF(EIGEN_RUNTIME_NO_MALLOC)

IF(EIGEN_NO_AUTOMATIC_RESIZING)
  MESSAGE(STATUS "Option EIGEN_NO_AUTOMATIC_RESIZING on.")
  ADD_DEFINITIONS(-DEIGEN_NO_AUTOMATIC_RESIZING)
ENDIF(EIGEN_NO_AUTOMATIC_RESIZING)

# ----------------------------------------------------
# --- DEPENDANCIES -----------------------------------
# ----------------------------------------------------
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.1.0") # Not sure about the needed version of Eigen, but you need Eigen::Ref, so 3.0.5 is not enough
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.2.0")

SET(BOOST_COMPONENTS unit_test_framework)

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
    include/pininvdyn/math/utils.hpp
    include/pininvdyn/math/constraint-base.hpp
    include/pininvdyn/math/constraint-equality.hpp
    include/pininvdyn/math/constraint-inequality.hpp
    include/pininvdyn/math/constraint-bound.hpp
  )

SET(${PROJECT_NAME}_TASKS_HEADERS
    include/pininvdyn/tasks/fwd.hpp
    include/pininvdyn/tasks/task-base.hpp
    include/pininvdyn/tasks/task-motion.hpp
    include/pininvdyn/tasks/task-actuation.hpp
    include/pininvdyn/tasks/task-contact-force.hpp
    include/pininvdyn/tasks/task-com-equality.hpp
    include/pininvdyn/tasks/task-se3-equality.hpp
    include/pininvdyn/tasks/task-contact-force-equality.hpp
    include/pininvdyn/tasks/task-actuation-equality.hpp
    include/pininvdyn/tasks/task-actuation-bounds.hpp
    include/pininvdyn/tasks/task-joint-bounds.hpp
    include/pininvdyn/tasks/task-joint-posture.hpp
  )

SET(${PROJECT_NAME}_CONTACTS_HEADERS
    include/pininvdyn/contacts/fwd.hpp
    include/pininvdyn/contacts/contact-base.hpp
    include/pininvdyn/contacts/contact-6d.hpp
  )

SET(${PROJECT_NAME}_TRAJECTORIES_HEADERS
    include/pininvdyn/trajectories/fwd.hpp
    include/pininvdyn/trajectories/trajectory-base.hpp
    include/pininvdyn/trajectories/trajectory-se3.hpp
    include/pininvdyn/trajectories/trajectory-euclidian.hpp
  )

SET(${PROJECT_NAME}_SOLVERS_HEADERS
    include/pininvdyn/solvers/eiquadprog_2011.hpp
    include/pininvdyn/solvers/eiquadprog_rt.h
    include/pininvdyn/solvers/eiquadprog_rt.hpp
    include/pininvdyn/solvers/eiquadprog_fast.h
    include/pininvdyn/solvers/eiquadprog_fast.hpp
    include/pininvdyn/solvers/fwd.hpp
    include/pininvdyn/solvers/solver-HQP-base.h
    include/pininvdyn/solvers/solver-HQP-base.hpp
    include/pininvdyn/solvers/solver-HQP-factory.hpp
    include/pininvdyn/solvers/solver-HQP-factory.hxx
    include/pininvdyn/solvers/solver-HQP-qpoases.hpp
    include/pininvdyn/solvers/solver-HQP-eiquadprog.hpp
    include/pininvdyn/solvers/solver-HQP-eiquadprog-rt.h
    include/pininvdyn/solvers/solver-HQP-eiquadprog-rt.hpp
    include/pininvdyn/solvers/solver-HQP-eiquadprog-fast.h
  )

SET(${PROJECT_NAME}_ROBOTS_HEADERS
    include/pininvdyn/robots/fwd.hpp
    include/pininvdyn/robots/robot-wrapper.hpp
  )

SET(${PROJECT_NAME}_FORMULATIONS_HEADERS
    include/pininvdyn/formulations/inverse-dynamics-formulation-base.hpp
    include/pininvdyn/formulations/inverse-dynamics-formulation-acc-force.hpp
  )

SET(HEADERS
    include/pininvdyn/config.hpp
    include/pininvdyn/utils/statistics.hpp
    include/pininvdyn/utils/stop-watch.hpp
    include/pininvdyn/utils/Stdafx.hh
    ${${PROJECT_NAME}_MATH_HEADERS}
    ${${PROJECT_NAME}_TASKS_HEADERS}
    ${${PROJECT_NAME}_CONTACTS_HEADERS}
    ${${PROJECT_NAME}_TRAJECTORIES_HEADERS}
    ${${PROJECT_NAME}_SOLVERS_HEADERS}
    ${${PROJECT_NAME}_ROBOTS_HEADERS}
    ${${PROJECT_NAME}_FORMULATIONS_HEADERS}
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
ADD_SUBDIRECTORY(unittest)

# --- PACKAGING ----------------------------------------------------------------
PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

SETUP_PROJECT_FINALIZE()
