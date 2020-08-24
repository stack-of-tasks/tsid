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
# tsid. If not, see
# <http://www.gnu.org/licenses/>.

# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------

MACRO(ADD_TEST_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TEST_CFLAGS)

MACRO(ADD_TESTCASE NAME)
  ADD_UNIT_TEST(${NAME} ${NAME}.cpp)

  ADD_TEST_CFLAGS(${NAME} "-DBOOST_TEST_DYN_LINK")
  SET(MODULE_NAME "${NAME}Test")
  STRING(REGEX REPLACE "-" "_" MODULE_NAME ${MODULE_NAME})
  ADD_TEST_CFLAGS(${NAME} "-DBOOST_TEST_MODULE=${MODULE_NAME}")

  TARGET_LINK_LIBRARIES(${NAME} PRIVATE ${PROJECT_NAME} Boost::unit_test_framework)
ENDMACRO(ADD_TESTCASE)

# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------

ADD_TESTCASE(constraints)
ADD_TESTCASE(trajectories)

ADD_TESTCASE(robot-wrapper)
ADD_TEST_CFLAGS(robot-wrapper '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

ADD_TESTCASE(tasks)
ADD_TEST_CFLAGS(tasks '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

ADD_TESTCASE(contacts)
ADD_TEST_CFLAGS(contacts '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

ADD_TESTCASE(tsid-formulation)
ADD_TEST_CFLAGS(tsid-formulation '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

ADD_TESTCASE(math_utils)
ADD_TESTCASE(hqp_solvers)

ADD_TESTCASE(set_gravity)
ADD_TEST_CFLAGS(set_gravity '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

IF(BUILD_PYTHON_INTERFACE)
  ADD_SUBDIRECTORY(python)
ENDIF(BUILD_PYTHON_INTERFACE)
