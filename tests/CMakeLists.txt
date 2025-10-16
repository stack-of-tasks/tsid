#
# Copyright (c) 2017-2020 CNRS
#

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

if(NOT BUILD_STANDALONE_PYTHON_INTERFACE)
  add_testcase(constraints)
  add_testcase(trajectories)

  add_testcase(robot-wrapper)
  add_test_cflags(robot-wrapper
                  '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

  add_testcase(tasks)
  add_test_cflags(tasks
                  '-DTSID_SOURCE_DIR=\\\"${${PROJECT_NAME}_SOURCE_DIR}\\\"')

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
endif()

if(BUILD_PYTHON_INTERFACE)
  add_subdirectory(python)
endif(BUILD_PYTHON_INTERFACE)
