#
# Copyright (c) 2017-2018 CNRS
#
# This file is part of tsid
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
INCLUDE(cmake/ide.cmake)
INCLUDE(cmake/apple.cmake)

SET(PROJECT_NAME tsid)
SET(PROJECT_DESCRIPTION "Efficient Task Space Inverse Dynamics for Multi-body Systems based on Pinocchio")
SET(PROJECT_URL "http://github.com/stack-of-tasks/tsid")
OPTION(INSTALL_DOCUMENTATION "Generate and install the documentation" OFF)

SET(DOXYGEN_USE_MATHJAX YES)

# Handle APPLE Cmake policy
IF(APPLE)
  APPLY_DEFAULT_APPLE_CONFIGURATION()
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
OPTION (BUILD_PYTHON_INTERFACE "Build the python binding" ON)
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
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.2.0") # Eigen::Ref appeared from 3.2.0
ADD_REQUIRED_DEPENDENCY("pinocchio >= 1.3.0")

SET(BOOST_REQUIERED_COMPONENTS filesystem system)
SET(BOOST_BUILD_COMPONENTS unit_test_framework)
SET(BOOST_OPTIONAL_COMPONENTS "")

IF(BUILD_PYTHON_INTERFACE)
  SET(BOOST_OPTIONAL_COMPONENTS ${BOOST_OPTIONAL_COMPONENTS} python)
  FINDPYTHON()
  INCLUDE_DIRECTORIES(SYSTEM ${PYTHON_INCLUDE_DIRS}) 
ENDIF(BUILD_PYTHON_INTERFACE)

SET(BOOST_COMPONENTS ${BOOST_REQUIERED_COMPONENTS} ${BOOST_OPTIONAL_COMPONENTS} ${BOOST_BUILD_COMPONENTS})
SEARCH_FOR_BOOST()

# Path to boost headers
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
# ----------------------------------------------------
# --- INCLUDE ----------------------------------------
# ----------------------------------------------------

SET(${PROJECT_NAME}_MATH_HEADERS
    include/tsid/math/fwd.hpp
    include/tsid/math/utils.hpp
    include/tsid/math/constraint-base.hpp
    include/tsid/math/constraint-equality.hpp
    include/tsid/math/constraint-inequality.hpp
    include/tsid/math/constraint-bound.hpp
  )

SET(${PROJECT_NAME}_TASKS_HEADERS
    include/tsid/tasks/fwd.hpp
    include/tsid/tasks/task-base.hpp
    include/tsid/tasks/task-motion.hpp
    include/tsid/tasks/task-actuation.hpp
    include/tsid/tasks/task-contact-force.hpp
    include/tsid/tasks/task-com-equality.hpp
    include/tsid/tasks/task-se3-equality.hpp
    include/tsid/tasks/task-contact-force-equality.hpp
    include/tsid/tasks/task-actuation-equality.hpp
    include/tsid/tasks/task-actuation-bounds.hpp
    include/tsid/tasks/task-joint-bounds.hpp
    include/tsid/tasks/task-joint-posture.hpp
  )

SET(${PROJECT_NAME}_CONTACTS_HEADERS
    include/tsid/contacts/fwd.hpp
    include/tsid/contacts/contact-base.hpp
    include/tsid/contacts/contact-6d.hpp
  )

SET(${PROJECT_NAME}_TRAJECTORIES_HEADERS
    include/tsid/trajectories/fwd.hpp
    include/tsid/trajectories/trajectory-base.hpp
    include/tsid/trajectories/trajectory-se3.hpp
    include/tsid/trajectories/trajectory-euclidian.hpp
  )

SET(${PROJECT_NAME}_SOLVERS_HEADERS
    include/tsid/solvers/eiquadprog_2011.hpp
    include/tsid/solvers/eiquadprog-rt.hpp
    include/tsid/solvers/eiquadprog-rt.hxx
    include/tsid/solvers/eiquadprog-fast.hpp
    include/tsid/solvers/fwd.hpp
    include/tsid/solvers/utils.hpp
    include/tsid/solvers/solver-HQP-output.hpp
    include/tsid/solvers/solver-HQP-base.hpp
    include/tsid/solvers/solver-HQP-factory.hpp
    include/tsid/solvers/solver-HQP-factory.hxx
    include/tsid/solvers/solver-HQP-qpoases.hpp
    include/tsid/solvers/solver-HQP-eiquadprog.hpp
    include/tsid/solvers/solver-HQP-eiquadprog-rt.hpp
    include/tsid/solvers/solver-HQP-eiquadprog-rt.hxx
    include/tsid/solvers/solver-HQP-eiquadprog-fast.hpp
  )

SET(${PROJECT_NAME}_ROBOTS_HEADERS
    include/tsid/robots/fwd.hpp
    include/tsid/robots/robot-wrapper.hpp
  )

SET(${PROJECT_NAME}_FORMULATIONS_HEADERS
    include/tsid/formulations/inverse-dynamics-formulation-base.hpp
    include/tsid/formulations/inverse-dynamics-formulation-acc-force.hpp
  )

SET(HEADERS
    include/tsid/config.hpp
    include/tsid/utils/statistics.hpp
    include/tsid/utils/stop-watch.hpp
    include/tsid/utils/Stdafx.hh
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
SET(PYWRAP ${PROJECT_NAME}_pywrap)
ADD_SUBDIRECTORY(bindings)

# --- UNIT TESTS ---------------------------------------------------------------
ADD_SUBDIRECTORY(unittest)

# --- PACKAGING ----------------------------------------------------------------
PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

SETUP_PROJECT_FINALIZE()
