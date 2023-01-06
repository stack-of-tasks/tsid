#
# Copyright (c) 2017-2021 CNRS
#
# This file is part of tsid tsid is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public License as
# published by the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version. tsid is distributed in the hope that it
# will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Lesser Public License for more details. You should have received a copy of the
# GNU Lesser General Public License along with tsid If not, see
# <http://www.gnu.org/licenses/>.

# --- LIBRARY --- #
file(
  GLOB
  ${PYWRAP}_SOURCES
  *.cpp
  constraint/*.cpp
  contacts/*.cpp
  formulations/*.cpp
  robots/*.cpp
  solvers/*.cpp
  tasks/*.cpp
  trajectories/*.cpp
  math/*.cpp
  utils/*.cpp
  measuredForces/*.cpp)

add_library(${PYWRAP} SHARED ${${PYWRAP}_SOURCES} ${${PYWRAP}_HEADERS})
target_link_libraries(${PYWRAP} PUBLIC ${PROJECT_NAME} eigenpy::eigenpy)

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

if(UNIX AND NOT APPLE)
  set_target_properties(${PYWRAP} PROPERTIES INSTALL_RPATH "\$ORIGIN/../../..")
endif()

install(
  TARGETS ${PYWRAP}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})

set(PYTHON_FILES __init__.py)

foreach(python ${PYTHON_FILES})
  python_build(${PROJECT_NAME} ${python})
  install(FILES "${${PROJECT_NAME}_SOURCE_DIR}/bindings/python/tsid/${python}"
          DESTINATION ${${PYWRAP}_INSTALL_DIR})
endforeach(python)
