#
# Copyright (c) 2017-2021 CNRS
#

# --- LIBRARY --- #
set(${PYWRAP}_SOURCES
    constraint/constraint-bound.cpp
    constraint/constraint-equality.cpp
    constraint/constraint-inequality.cpp
    contacts/contact-6d.cpp
    contacts/contact-point.cpp
    contacts/contact-two-frame-positions.cpp
    contacts/measured-6d-wrench.cpp
    formulations/formulation.cpp
    math/utils.cpp
    module.cpp
    robots/robot-wrapper.cpp
    solvers/HQPData.cpp
    solvers/HQPOutput.cpp
    solvers/solver-HQP-eiquadprog.cpp
    tasks/task-actuation-bounds.cpp
    tasks/task-actuation-equality.cpp
    tasks/task-am-equality.cpp
    tasks/task-com-equality.cpp
    tasks/task-contact-force-equality.cpp
    tasks/task-cop-equality.cpp
    tasks/task-joint-bounds.cpp
    tasks/task-joint-posture.cpp
    tasks/task-joint-posVelAcc-bounds.cpp
    tasks/task-se3-equality.cpp
    tasks/task-two-frames-equality.cpp
    trajectories/trajectory-base.cpp
    trajectories/trajectory-euclidian.cpp
    trajectories/trajectory-se3.cpp)

set(${PYWRAP}_HEADERS
    ../../include/tsid/bindings/python/constraint/constraint-bound.hpp
    ../../include/tsid/bindings/python/constraint/constraint-equality.hpp
    ../../include/tsid/bindings/python/constraint/constraint-inequality.hpp
    ../../include/tsid/bindings/python/constraint/expose-constraints.hpp
    ../../include/tsid/bindings/python/contacts/contact-6d.hpp
    ../../include/tsid/bindings/python/contacts/contact-point.hpp
    ../../include/tsid/bindings/python/contacts/contact-two-frame-positions.hpp
    ../../include/tsid/bindings/python/contacts/contact-two-frames.hpp
    ../../include/tsid/bindings/python/contacts/measured-6d-wrench.hpp
    ../../include/tsid/bindings/python/contacts/expose-contact.hpp
    ../../include/tsid/bindings/python/formulations/expose-formulations.hpp
    ../../include/tsid/bindings/python/formulations/formulation.hpp
    ../../include/tsid/bindings/python/fwd.hpp
    ../../include/tsid/bindings/python/math/utils.hpp
    ../../include/tsid/bindings/python/robots/expose-robots.hpp
    ../../include/tsid/bindings/python/robots/robot-wrapper.hpp
    ../../include/tsid/bindings/python/solvers/expose-solvers.hpp
    ../../include/tsid/bindings/python/solvers/HQPData.hpp
    ../../include/tsid/bindings/python/solvers/HQPOutput.hpp
    ../../include/tsid/bindings/python/solvers/solver-HQP-eiquadprog.hpp
    ../../include/tsid/bindings/python/solvers/solver-osqp.hpp
    ../../include/tsid/bindings/python/solvers/solver-proxqp.hpp
    ../../include/tsid/bindings/python/tasks/expose-tasks.hpp
    ../../include/tsid/bindings/python/tasks/task-actuation-bounds.hpp
    ../../include/tsid/bindings/python/tasks/task-am-equality.hpp
    ../../include/tsid/bindings/python/tasks/task-com-equality.hpp
    ../../include/tsid/bindings/python/tasks/task-contact-force-equality.hpp
    ../../include/tsid/bindings/python/tasks/task-cop-equality.hpp
    ../../include/tsid/bindings/python/tasks/task-joint-bounds.hpp
    ../../include/tsid/bindings/python/tasks/task-joint-posture.hpp
    ../../include/tsid/bindings/python/tasks/task-joint-posVelAcc-bounds.hpp
    ../../include/tsid/bindings/python/tasks/task-se3-equality.hpp
    ../../include/tsid/bindings/python/tasks/task-two-frames-equality.hpp
    ../../include/tsid/bindings/python/trajectories/expose-trajectories.hpp
    ../../include/tsid/bindings/python/trajectories/trajectory-base.hpp
    ../../include/tsid/bindings/python/trajectories/trajectory-euclidian.hpp
    ../../include/tsid/bindings/python/trajectories/trajectory-se3.hpp
    ../../include/tsid/bindings/python/utils/container.hpp)

add_library(${PYWRAP} SHARED ${${PYWRAP}_SOURCES} ${${PYWRAP}_HEADERS})
target_link_libraries(${PYWRAP} PUBLIC ${PROJECT_NAME}::${PROJECT_NAME}
                                       eigenpy::eigenpy)

if(IS_ABSOLUTE ${PYTHON_SITELIB})
  set(${PYWRAP}_INSTALL_DIR ${PYTHON_SITELIB}/${PROJECT_NAME})
else()
  set(${PYWRAP}_INSTALL_DIR
      ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB}/${PROJECT_NAME})
endif()

if(BUILD_WITH_COMMIT_VERSION)
  tag_library_version(${PYWRAP})
endif(BUILD_WITH_COMMIT_VERSION)
add_header_group(${PYWRAP}_HEADERS)
add_source_group(${PYWRAP}_SOURCES)

# We need to change the extension for python bindings
set_target_properties(
  ${PYWRAP}
  PROPERTIES SUFFIX ${PYTHON_EXT_SUFFIX}
             PREFIX ""
             LIBRARY_OUTPUT_DIRECTORY
             "${CMAKE_BINARY_DIR}/bindings/python/${PROJECT_NAME}")

if(UNIX)
  get_relative_rpath(${${PYWRAP}_INSTALL_DIR} ${PYWRAP}_INSTALL_RPATH)
  set_target_properties(${PYWRAP} PROPERTIES INSTALL_RPATH
                                             "${${PYWRAP}_INSTALL_RPATH}")
endif()

install(
  TARGETS ${PYWRAP}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})

set(PYTHON_FILES __init__.py)

foreach(python ${PYTHON_FILES})
  python_install_on_site(${PROJECT_NAME} ${python})
endforeach(python)
