#
# Copyright (c) 2017-2018 CNRS
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

# --- MAIN LIBRARY --- #
SET(LIBRARY_NAME ${PROJECT_NAME})

SET(${LIBRARY_NAME}_MATH_SOURCES
    math/constraint-base.cpp
    math/constraint-equality.cpp
    math/constraint-inequality.cpp
    math/constraint-bound.cpp
    math/utils.cpp
  )

SET(${LIBRARY_NAME}_TASKS_SOURCES
    tasks/task-base.cpp
    tasks/task-actuation-bounds.cpp
    tasks/task-actuation-equality.cpp
    tasks/task-actuation.cpp
    tasks/task-com-equality.cpp
    tasks/task-contact-force-equality.cpp
    tasks/task-contact-force.cpp
    tasks/task-joint-bounds.cpp
    tasks/task-joint-posture.cpp
    tasks/task-motion.cpp
    tasks/task-se3-equality.cpp
  )

SET(${LIBRARY_NAME}_CONTACTS_SOURCES
    contacts/contact-base.cpp
    contacts/contact-6d.cpp
    contacts/contact-point.cpp
  )

SET(${LIBRARY_NAME}_TRAJECTORIES_SOURCES
    trajectories/trajectory-se3.cpp
    trajectories/trajectory-euclidian.cpp
  )

SET(${LIBRARY_NAME}_SOLVERS_SOURCES
    solvers/eiquadprog-fast.cpp
    solvers/solver-HQP-base.cpp
    solvers/solver-HQP-factory.cpp
    solvers/solver-HQP-eiquadprog.cpp
    solvers/solver-HQP-eiquadprog-fast.cpp
    solvers/solver-HQP-qpoases.cpp
    solvers/utils.cpp
  )

SET(${LIBRARY_NAME}_ROBOTS_SOURCES
    robots/robot-wrapper.cpp
  )

SET(${LIBRARY_NAME}_FORMULATIONS_SOURCES
    formulations/inverse-dynamics-formulation-base.cpp
    formulations/inverse-dynamics-formulation-acc-force.cpp
  )

SET(${LIBRARY_NAME}_SOURCES
    utils/statistics.cpp
    utils/stop-watch.cpp
    ${${LIBRARY_NAME}_MATH_SOURCES}
    ${${LIBRARY_NAME}_TASKS_SOURCES}
    ${${LIBRARY_NAME}_CONTACTS_SOURCES}
    ${${LIBRARY_NAME}_TRAJECTORIES_SOURCES}
    ${${LIBRARY_NAME}_SOLVERS_SOURCES}
    ${${LIBRARY_NAME}_ROBOTS_SOURCES}
    ${${LIBRARY_NAME}_FORMULATIONS_SOURCES}
  )

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${HEADERS_FULL_PATH} ${${LIBRARY_NAME}_SOURCES})
SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES LINKER_LANGUAGE CXX)
ADD_HEADER_GROUP(HEADERS_FULL_PATH)
ADD_SOURCE_GROUP(${LIBRARY_NAME}_SOURCES)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} eigen3)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} pinocchio)

INSTALL(TARGETS ${PROJECT_NAME} DESTINATION lib)

