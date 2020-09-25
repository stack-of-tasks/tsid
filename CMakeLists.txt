#
# Copyright (c) 2017-2020 CNRS
#
# This file is part of tsid
# tsid is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
# tsid is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Lesser Public License for more details. You should have
# received a copy of the GNU Lesser General Public License along with
# tsid If not, see
# <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

# Project properties
SET(PROJECT_ORG stack-of-tasks)
SET(PROJECT_NAME tsid)
SET(PROJECT_DESCRIPTION "Efficient Task Space Inverse Dynamics for Multi-body Systems based on Pinocchio")
SET(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
OPTION(BUILD_PYTHON_INTERFACE "Build the python bindings" ON)
OPTION(INSTALL_PYTHON_INTERFACE_ONLY "Install *ONLY* the python bindings" OFF)
OPTION(SUFFIX_SO_VERSION "Suffix library name with its version" ON)
OPTION(INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" OFF)
OPTION(EIGEN_RUNTIME_NO_MALLOC "If ON, it can assert in case of runtime allocation" ON)
OPTION(EIGEN_NO_AUTOMATIC_RESIZING "If ON, it forbids automatic resizing of dynamics arrays and matrices" OFF)

# Project configuration
IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  SET(PROJECT_USE_CMAKE_EXPORT TRUE)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
SET(CXX_DISABLE_WERROR TRUE)
SET(DOXYGEN_USE_MATHJAX YES)
SET(CMAKE_VERBOSE_MAKEFILE TRUE)

# JRL-cmakemodule setup
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/python.cmake)
INCLUDE(cmake/ide.cmake)
INCLUDE(cmake/apple.cmake)

# Handle APPLE Cmake policy
IF(APPLE)
  APPLY_DEFAULT_APPLE_CONFIGURATION()
ENDIF(APPLE)

# Project definition
COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

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

CHECK_MINIMAL_CXX_STANDARD(11 ENFORCE)

# Project dependencies
ADD_PROJECT_DEPENDENCY(pinocchio 2.3.1 REQUIRED
  PKG_CONFIG_REQUIRES "pinocchio >= 2.3.1")
ADD_PROJECT_DEPENDENCY(eiquadprog 1.1.3 REQUIRED
  PKG_CONFIG_REQUIRES "eiquadprog >= 1.1.3")

IF(BUILD_TESTING)
  FIND_PACKAGE(Boost REQUIRED COMPONENTS unit_test_framework)
ENDIF(BUILD_TESTING)

IF(BUILD_PYTHON_INTERFACE)
  FINDPYTHON()
  SET(PYWRAP ${PROJECT_NAME}_pywrap)
  ADD_PROJECT_DEPENDENCY(eigenpy REQUIRED PKG_CONFIG_REQUIRES eigenpy)
  SEARCH_FOR_BOOST_PYTHON(REQUIRED)
ENDIF(BUILD_PYTHON_INTERFACE)

# Main Library
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
  include/tsid/tasks/task-joint-posVelAcc-bounds.hpp
  include/tsid/tasks/task-capture-point-inequality.hpp
  include/tsid/tasks/task-angular-momentum-equality.hpp
  )

SET(${PROJECT_NAME}_CONTACTS_HEADERS
  include/tsid/contacts/fwd.hpp
  include/tsid/contacts/contact-base.hpp
  include/tsid/contacts/contact-6d.hpp
  include/tsid/contacts/contact-point.hpp
  )

SET(${PROJECT_NAME}_TRAJECTORIES_HEADERS
  include/tsid/trajectories/fwd.hpp
  include/tsid/trajectories/trajectory-base.hpp
  include/tsid/trajectories/trajectory-se3.hpp
  include/tsid/trajectories/trajectory-euclidian.hpp
  )

SET(${PROJECT_NAME}_SOLVERS_HEADERS
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

FILE(GLOB ${PYWRAP}_HEADERS
  include/tsid/bindings/python/constraint/*.hpp
  include/tsid/bindings/python/contacts/*.hpp
  include/tsid/bindings/python/formulations/*.hpp
  include/tsid/bindings/python/robots/*.hpp
  include/tsid/bindings/python/solvers/*.hpp
  include/tsid/bindings/python/tasks/*.hpp
  include/tsid/bindings/python/trajectories/*.hpp
  include/tsid/bindings/python/utils/*.hpp
  )

SET(${PROJECT_NAME}_HEADERS
  include/tsid/config.hpp
  include/tsid/deprecation.hpp
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

LIST(REMOVE_DUPLICATES ${PROJECT_NAME}_HEADERS)

SET(${PROJECT_NAME}_MATH_SOURCES
  src/math/constraint-base.cpp
  src/math/constraint-equality.cpp
  src/math/constraint-inequality.cpp
  src/math/constraint-bound.cpp
  src/math/utils.cpp
  )

SET(${PROJECT_NAME}_TASKS_SOURCES
  src/tasks/task-base.cpp
  src/tasks/task-actuation-bounds.cpp
  src/tasks/task-actuation-equality.cpp
  src/tasks/task-actuation.cpp
  src/tasks/task-com-equality.cpp
  src/tasks/task-contact-force-equality.cpp
  src/tasks/task-contact-force.cpp
  src/tasks/task-joint-bounds.cpp
  src/tasks/task-joint-posture.cpp
  src/tasks/task-joint-posVelAcc-bounds.cpp
  src/tasks/task-capture-point-inequality.cpp
  src/tasks/task-motion.cpp
  src/tasks/task-se3-equality.cpp
  src/tasks/task-angular-momentum-equality.cpp
  )

SET(${PROJECT_NAME}_CONTACTS_SOURCES
  src/contacts/contact-base.cpp
  src/contacts/contact-6d.cpp
  src/contacts/contact-point.cpp
  )

SET(${PROJECT_NAME}_TRAJECTORIES_SOURCES
  src/trajectories/trajectory-se3.cpp
  src/trajectories/trajectory-euclidian.cpp
  )

SET(${PROJECT_NAME}_SOLVERS_SOURCES
  src/solvers/solver-HQP-base.cpp
  src/solvers/solver-HQP-factory.cpp
  src/solvers/solver-HQP-eiquadprog.cpp
  src/solvers/solver-HQP-eiquadprog-fast.cpp
  src/solvers/solver-HQP-qpoases.cpp
  src/solvers/utils.cpp
  )

SET(${PROJECT_NAME}_ROBOTS_SOURCES
  src/robots/robot-wrapper.cpp
  )

SET(${PROJECT_NAME}_FORMULATIONS_SOURCES
  src/formulations/inverse-dynamics-formulation-base.cpp
  src/formulations/inverse-dynamics-formulation-acc-force.cpp
  )

SET(${PROJECT_NAME}_SOURCES
  src/utils/statistics.cpp
  src/utils/stop-watch.cpp
  ${${PROJECT_NAME}_MATH_SOURCES}
  ${${PROJECT_NAME}_TASKS_SOURCES}
  ${${PROJECT_NAME}_CONTACTS_SOURCES}
  ${${PROJECT_NAME}_TRAJECTORIES_SOURCES}
  ${${PROJECT_NAME}_SOLVERS_SOURCES}
  ${${PROJECT_NAME}_ROBOTS_SOURCES}
  ${${PROJECT_NAME}_FORMULATIONS_SOURCES}
  )

ADD_HEADER_GROUP(${PROJECT_NAME}_HEADERS)
ADD_SOURCE_GROUP(${PROJECT_NAME}_SOURCES)

ADD_LIBRARY(${PROJECT_NAME} SHARED
  ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} PUBLIC
  pinocchio::pinocchio eiquadprog::eiquadprog)


IF(SUFFIX_SO_VERSION)
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES SOVERSION ${PROJECT_VERSION})
ENDIF(SUFFIX_SO_VERSION)

IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)

ADD_SUBDIRECTORY(bindings)
ADD_SUBDIRECTORY(tests)

# --- PACKAGING ----------------------------------------------------------------
PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})
IF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
  INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
ENDIF(NOT INSTALL_PYTHON_INTERFACE_ONLY)
