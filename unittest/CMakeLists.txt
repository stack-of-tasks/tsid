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
# tsid. If not, see
# <http://www.gnu.org/licenses/>.

# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------

IF(BUILD_UNIT_TESTS)
  SET(DISABLE_TESTS OFF)
ELSE(BUILD_UNIT_TESTS)
  SET(DISABLE_TESTS ON)
ENDIF(BUILD_UNIT_TESTS)
include(../cmake/test.cmake)

MACRO(ADD_TEST_CFLAGS target flag)
  SET_PROPERTY(TARGET ${target} APPEND_STRING PROPERTY COMPILE_FLAGS " ${flag}")
ENDMACRO(ADD_TEST_CFLAGS)

MACRO(ADD_TESTCASE NAME PKGS)
  ADD_UNIT_TEST(${NAME} ${NAME})

  SET_TARGET_PROPERTIES(${NAME} PROPERTIES LINKER_LANGUAGE CXX)
  ADD_TEST_CFLAGS(${NAME} "-DBOOST_TEST_DYN_LINK") 
  SET(MODULE_NAME "${NAME}Test")
  STRING(REGEX REPLACE "-" "_" MODULE_NAME ${MODULE_NAME})
  ADD_TEST_CFLAGS(${NAME} "-DBOOST_TEST_MODULE=${MODULE_NAME}") 

  FOREACH(PKG ${PKGS})
    PKG_CONFIG_USE_DEPENDENCY(${NAME} ${PKG})
  ENDFOREACH(PKG)

  TARGET_LINK_LIBRARIES(${NAME} ${PROJECT_NAME})
  TARGET_LINK_LIBRARIES(${NAME} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
ENDMACRO(ADD_TESTCASE)

# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------

ADD_TESTCASE(constraints "eigen3;pinocchio")
ADD_TESTCASE(trajectories "eigen3;pinocchio")

ADD_TESTCASE(robot-wrapper "eigen3;pinocchio")
ADD_TEST_CFLAGS(robot-wrapper '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

ADD_TESTCASE(tasks "eigen3;pinocchio")
ADD_TEST_CFLAGS(tasks '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

ADD_TESTCASE(contacts "eigen3;pinocchio")
ADD_TEST_CFLAGS(contacts '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

ADD_TESTCASE(tsid-formulation "eigen3;pinocchio")
ADD_TEST_CFLAGS(tsid-formulation '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

ADD_TESTCASE(math_utils "eigen3;pinocchio")
ADD_TESTCASE(hqp_solvers "eigen3;pinocchio")
