#
# Copyright (c) 2017-2020 CNRS
#
# This file is part of tsid tsid is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version. tsid is distributed in the hope that it
# will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Lesser Public License for more details. You should have received a copy of the
# GNU Lesser General Public License along with tsid. If not, see
# <http://www.gnu.org/licenses/>.

# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------
# --- MACROS ------------------------------------------------------------------

macro(ADD_TEST_CFLAGS target flag)
  set_property(
    TARGET ${target}
    APPEND_STRING
    PROPERTY COMPILE_FLAGS " ${flag}")
endmacro(ADD_TEST_CFLAGS)

macro(ADD_TESTCASE NAME)
  add_unit_test(${NAME} ${NAME}.cpp)

  add_test_cflags(${NAME} "-DBOOST_TEST_DYN_LINK")
  set(MODULE_NAME "${NAME}Test")
  string(REGEX REPLACE "-" "_" MODULE_NAME ${MODULE_NAME})
  add_test_cflags(${NAME} "-DBOOST_TEST_MODULE=${MODULE_NAME}")

  target_link_libraries(${NAME} PRIVATE ${PROJECT_NAME}
                                        Boost::unit_test_framework)
endmacro(ADD_TESTCASE)

# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------
# --- RULES -------------------------------------------------------------------

add_testcase(constraints)
add_testcase(trajectories)

add_testcase(robot-wrapper)
add_test_cflags(robot-wrapper
                '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

add_testcase(tasks)
add_test_cflags(tasks '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

add_testcase(contacts)
add_test_cflags(contacts
                '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

add_testcase(tsid-formulation)
add_test_cflags(tsid-formulation
                '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

add_testcase(math_utils)
add_testcase(hqp_solvers)

add_testcase(set_gravity)
add_test_cflags(set_gravity
                '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif(BUILD_PYTHON_INTERFACE)
