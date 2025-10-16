#
# Copyright (c) 2017-2021 CNRS
#

cmake_minimum_required(VERSION 3.22)

# Project properties
set(PROJECT_ORG stack-of-tasks)
set(PROJECT_NAME tsid)
set(PROJECT_DESCRIPTION
    "Efficient Task Space Inverse Dynamics for Multi-body Systems based on Pinocchio"
)
set(PROJECT_URL "https://github.com/${PROJECT_ORG}/${PROJECT_NAME}")

# Project options
option(INSTALL_DOCUMENTATION "Build and install the documentation" OFF)
option(SUFFIX_SO_VERSION "Suffix library name with its version" ON)
option(INITIALIZE_WITH_NAN "Initialize Eigen entries with NaN" OFF)
option(EIGEN_RUNTIME_NO_MALLOC
       "If ON, it can assert in case of runtime allocation" ON)
option(EIGEN_NO_AUTOMATIC_RESIZING
       "If ON, it forbids automatic resizing of dynamics arrays and matrices"
       OFF)
option(BUILD_WITH_PROXQP "Support using the proxqp solver" OFF)
option(BUILD_WITH_OSQP "Support using the osqp solver" OFF)

# With pos, vel, acc awaiting renaming (e.g. in trajectory-base), we are
# producing a ton of deprecation warnings. Ignoring them for now; remove this
# once pos, vel, acc are renamed.
add_definitions(-Wno-deprecated-declarations)

# We frequently convert between signed and unsigned integers for Eigen::Index.
# Ignore them for now.
add_definitions(-Wno-sign-conversion)

add_definitions(-Werror=suggest-override)

# Project configuration
if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  set(PROJECT_USE_CMAKE_EXPORT TRUE)
endif()
set(CXX_DISABLE_WERROR TRUE)
set(DOXYGEN_USE_MATHJAX YES)
set(CMAKE_VERBOSE_MAKEFILE TRUE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
  find_package(jrl-cmakemodules QUIET CONFIG)
  if(jrl-cmakemodules_FOUND)
    get_property(
      JRL_CMAKE_MODULES
      TARGET jrl-cmakemodules::jrl-cmakemodules
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

include("${JRL_CMAKE_MODULES}/base.cmake")
include("${JRL_CMAKE_MODULES}/ide.cmake")
include("${JRL_CMAKE_MODULES}/apple.cmake")

# Handle APPLE Cmake policy
if(APPLE)
  apply_default_apple_configuration()
endif(APPLE)

# Project definition
compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

if(INITIALIZE_WITH_NAN)
  message(STATUS "Initialize with NaN all the Eigen entries.")
  add_definitions(-DEIGEN_INITIALIZE_MATRICES_BY_NAN)
endif(INITIALIZE_WITH_NAN)

if(EIGEN_RUNTIME_NO_MALLOC)
  message(STATUS "Option EIGEN_RUNTIME_NO_MALLOC on.")
  add_definitions(-DEIGEN_RUNTIME_NO_MALLOC)
endif(EIGEN_RUNTIME_NO_MALLOC)

if(EIGEN_NO_AUTOMATIC_RESIZING)
  message(STATUS "Option EIGEN_NO_AUTOMATIC_RESIZING on.")
  add_definitions(-DEIGEN_NO_AUTOMATIC_RESIZING)
endif(EIGEN_NO_AUTOMATIC_RESIZING)

check_minimal_cxx_standard(17 ENFORCE)

# Project dependencies
if(BUILD_PYTHON_INTERFACE)
  set(PYWRAP ${PROJECT_NAME}_pywrap)
  add_project_dependency(eigenpy 2.7.12 REQUIRED)
  if(BUILD_STANDALONE_PYTHON_INTERFACE)
    add_project_dependency(${PROJECT_NAME} REQUIRED CONFIG)
  endif()
endif(BUILD_PYTHON_INTERFACE)

add_project_dependency(pinocchio 2.3.1 REQUIRED)
add_project_dependency(eiquadprog 1.1.3 REQUIRED)

find_package(qpmad QUIET) # optional
if(qpmad_FOUND)
  message(STATUS "qpmad found - building with qpmad support")
  add_project_dependency(qpmad QUIET)
endif()

if(BUILD_TESTING)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
endif(BUILD_TESTING)

# Main Library
set(${PROJECT_NAME}_MATH_HEADERS
    include/tsid/math/fwd.hpp
    include/tsid/math/utils.hpp
    include/tsid/math/constraint-base.hpp
    include/tsid/math/constraint-equality.hpp
    include/tsid/math/constraint-inequality.hpp
    include/tsid/math/constraint-bound.hpp)

set(${PROJECT_NAME}_TASKS_HEADERS
    include/tsid/tasks/fwd.hpp
    include/tsid/tasks/task-base.hpp
    include/tsid/tasks/task-motion.hpp
    include/tsid/tasks/task-actuation.hpp
    include/tsid/tasks/task-contact-force.hpp
    include/tsid/tasks/task-com-equality.hpp
    include/tsid/tasks/task-se3-equality.hpp
    include/tsid/tasks/task-contact-force-equality.hpp
    include/tsid/tasks/task-cop-equality.hpp
    include/tsid/tasks/task-actuation-equality.hpp
    include/tsid/tasks/task-actuation-bounds.hpp
    include/tsid/tasks/task-joint-bounds.hpp
    include/tsid/tasks/task-joint-posture.hpp
    include/tsid/tasks/task-joint-posVelAcc-bounds.hpp
    include/tsid/tasks/task-capture-point-inequality.hpp
    include/tsid/tasks/task-angular-momentum-equality.hpp
    include/tsid/tasks/task-two-frames-equality.hpp)

set(${PROJECT_NAME}_CONTACTS_HEADERS
    include/tsid/contacts/fwd.hpp
    include/tsid/contacts/contact-base.hpp
    include/tsid/contacts/contact-6d.hpp
    include/tsid/contacts/contact-point.hpp
    include/tsid/contacts/measured-force-base.hpp
    include/tsid/contacts/measured-3d-force.hpp
    include/tsid/contacts/measured-6d-wrench.hpp
    include/tsid/contacts/measured-3Dforce.hpp
    include/tsid/contacts/measured-6Dwrench.hpp
    include/tsid/contacts/contact-two-frame-positions.hpp)

set(${PROJECT_NAME}_TRAJECTORIES_HEADERS
    include/tsid/trajectories/fwd.hpp
    include/tsid/trajectories/trajectory-base.hpp
    include/tsid/trajectories/trajectory-se3.hpp
    include/tsid/trajectories/trajectory-euclidian.hpp)

set(${PROJECT_NAME}_SOLVERS_HEADERS
    include/tsid/solvers/fwd.hpp
    include/tsid/solvers/utils.hpp
    include/tsid/solvers/solver-qpData.hpp
    include/tsid/solvers/solver-HQP-output.hpp
    include/tsid/solvers/solver-HQP-base.hpp
    include/tsid/solvers/solver-HQP-factory.hpp
    include/tsid/solvers/solver-HQP-factory.hxx
    include/tsid/solvers/solver-HQP-qpoases.hpp
    include/tsid/solvers/solver-HQP-eiquadprog.hpp
    include/tsid/solvers/solver-HQP-eiquadprog-rt.hpp
    include/tsid/solvers/solver-HQP-eiquadprog-rt.hxx
    include/tsid/solvers/solver-HQP-eiquadprog-fast.hpp)

if(BUILD_WITH_PROXQP)
  find_package(proxsuite REQUIRED)
  list(APPEND ${PROJECT_NAME}_SOLVERS_HEADERS
       include/tsid/solvers/solver-proxqp.hpp)

  # Determine whether ot use the default target or the vectorized target, if
  # available
  set(proxsuite_INTERFACE proxsuite::proxsuite)
  if(NOT TARGET proxsuite::proxsuite-vectorized)
    message(
      STATUS
        "proxsuite::proxsuite-vectorized not available - defaulting to non-vectorized ProxQP"
    )
  else()
    set(proxsuite_INTERFACE proxsuite::proxsuite-vectorized)
  endif()
endif()

if(BUILD_WITH_OSQP)
  find_package(OsqpEigen REQUIRED)
  list(APPEND ${PROJECT_NAME}_SOLVERS_HEADERS
       include/tsid/solvers/solver-osqp.hpp)
endif()

if(qpmad_FOUND)
  list(APPEND ${PROJECT_NAME}_SOLVERS_HEADERS
       include/tsid/solvers/solver-HQP-qpmad.hpp)
endif()

set(${PROJECT_NAME}_ROBOTS_HEADERS include/tsid/robots/fwd.hpp
                                   include/tsid/robots/robot-wrapper.hpp)

set(${PROJECT_NAME}_FORMULATIONS_HEADERS
    include/tsid/formulations/contact-level.hpp
    include/tsid/formulations/inverse-dynamics-formulation-base.hpp
    include/tsid/formulations/inverse-dynamics-formulation-acc-force.hpp)

set(${PROJECT_NAME}_HEADERS
    include/tsid/macros.hpp
    include/tsid/utils/statistics.hpp
    include/tsid/utils/stop-watch.hpp
    include/tsid/utils/Stdafx.hh
    ${${PROJECT_NAME}_MATH_HEADERS}
    ${${PROJECT_NAME}_TASKS_HEADERS}
    ${${PROJECT_NAME}_CONTACTS_HEADERS}
    ${${PROJECT_NAME}_TRAJECTORIES_HEADERS}
    ${${PROJECT_NAME}_SOLVERS_HEADERS}
    ${${PROJECT_NAME}_ROBOTS_HEADERS}
    ${${PROJECT_NAME}_FORMULATIONS_HEADERS})

list(REMOVE_DUPLICATES ${PROJECT_NAME}_HEADERS)

set(${PROJECT_NAME}_MATH_SOURCES
    src/math/constraint-base.cpp src/math/constraint-equality.cpp
    src/math/constraint-inequality.cpp src/math/constraint-bound.cpp
    src/math/utils.cpp)

set(${PROJECT_NAME}_TASKS_SOURCES
    src/tasks/task-base.cpp
    src/tasks/task-actuation-bounds.cpp
    src/tasks/task-actuation-equality.cpp
    src/tasks/task-actuation.cpp
    src/tasks/task-com-equality.cpp
    src/tasks/task-contact-force-equality.cpp
    src/tasks/task-contact-force.cpp
    src/tasks/task-cop-equality.cpp
    src/tasks/task-joint-bounds.cpp
    src/tasks/task-joint-posture.cpp
    src/tasks/task-joint-posVelAcc-bounds.cpp
    src/tasks/task-capture-point-inequality.cpp
    src/tasks/task-motion.cpp
    src/tasks/task-se3-equality.cpp
    src/tasks/task-angular-momentum-equality.cpp
    src/tasks/task-two-frames-equality.cpp)

set(${PROJECT_NAME}_CONTACTS_SOURCES
    src/contacts/contact-base.cpp
    src/contacts/contact-6d.cpp
    src/contacts/contact-point.cpp
    src/contacts/measured-force-base.cpp
    src/contacts/measured-3d-force.cpp
    src/contacts/measured-6d-wrench.cpp
    src/contacts/contact-two-frame-positions.cpp)

set(${PROJECT_NAME}_TRAJECTORIES_SOURCES
    src/trajectories/trajectory-se3.cpp
    src/trajectories/trajectory-euclidian.cpp)

set(${PROJECT_NAME}_SOLVERS_SOURCES
    src/solvers/solver-HQP-base.cpp
    src/solvers/solver-HQP-factory.cpp
    src/solvers/solver-HQP-eiquadprog.cpp
    src/solvers/solver-HQP-eiquadprog-fast.cpp
    src/solvers/solver-HQP-qpoases.cpp
    src/solvers/utils.cpp)

if(BUILD_WITH_PROXQP)
  list(APPEND ${PROJECT_NAME}_SOLVERS_SOURCES src/solvers/solver-proxqp.cpp)
endif()

if(BUILD_WITH_OSQP)
  list(APPEND ${PROJECT_NAME}_SOLVERS_SOURCES src/solvers/solver-osqp.cpp)
endif()

if(qpmad_FOUND)
  list(APPEND ${PROJECT_NAME}_SOLVERS_SOURCES src/solvers/solver-HQP-qpmad.cpp)
endif()

set(${PROJECT_NAME}_ROBOTS_SOURCES src/robots/robot-wrapper.cpp)

set(${PROJECT_NAME}_FORMULATIONS_SOURCES
    src/formulations/contact-level.cpp
    src/formulations/inverse-dynamics-formulation-base.cpp
    src/formulations/inverse-dynamics-formulation-acc-force.cpp)

set(${PROJECT_NAME}_SOURCES
    src/utils/statistics.cpp
    src/utils/stop-watch.cpp
    ${${PROJECT_NAME}_MATH_SOURCES}
    ${${PROJECT_NAME}_TASKS_SOURCES}
    ${${PROJECT_NAME}_CONTACTS_SOURCES}
    ${${PROJECT_NAME}_TRAJECTORIES_SOURCES}
    ${${PROJECT_NAME}_SOLVERS_SOURCES}
    ${${PROJECT_NAME}_ROBOTS_SOURCES}
    ${${PROJECT_NAME}_FORMULATIONS_SOURCES})

add_header_group(${PROJECT_NAME}_HEADERS)
add_source_group(${PROJECT_NAME}_SOURCES)

if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                     ${${PROJECT_NAME}_HEADERS})
  add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
  target_include_directories(
    ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
  target_link_libraries(${PROJECT_NAME} PUBLIC pinocchio::pinocchio
                                               eiquadprog::eiquadprog)

  if(APPLE)
    set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH
                                                     "@loader_path")
  elseif(UNIX)
    set_target_properties(${PROJECT_NAME} PROPERTIES INSTALL_RPATH "\$ORIGIN")
  endif()

  if(BUILD_WITH_PROXQP)
    target_compile_definitions(${PROJECT_NAME} PUBLIC -DTSID_WITH_PROXSUITE)
    target_link_libraries(${PROJECT_NAME} PUBLIC proxsuite::proxsuite)
  endif()

  if(BUILD_WITH_OSQP)
    target_link_libraries(${PROJECT_NAME} PUBLIC OsqpEigen::OsqpEigen)
    target_compile_definitions(${PROJECT_NAME} PUBLIC -DTSID_WITH_OSQP)
  endif()

  if(qpmad_FOUND)
    target_include_directories(${PROJECT_NAME} PUBLIC ${qpmad_INCLUDE_DIRS})
    target_compile_definitions(${PROJECT_NAME} PUBLIC -DTSID_QPMAD_FOUND)
  endif()

  if(SUFFIX_SO_VERSION)
    set_target_properties(${PROJECT_NAME} PROPERTIES SOVERSION
                                                     ${PROJECT_VERSION})
  endif(SUFFIX_SO_VERSION)

  install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${TARGETS_EXPORT_NAME}
    DESTINATION lib)
endif()

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(bindings)
endif()
if(BUILD_TESTING)
  add_subdirectory(tests)
endif(BUILD_TESTING)
