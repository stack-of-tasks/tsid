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
  utils/*.cpp)

add_library(${PYWRAP} SHARED ${${PYWRAP}_SOURCES} ${${PYWRAP}_HEADERS})
target_link_libraries(${PYWRAP} PUBLIC ${PROJECT_NAME} eigenpy::eigenpy)

if(BUILD_WITH_COMMIT_VERSION)
  tag_library_version(${PYWRAP})
endif(BUILD_WITH_COMMIT_VERSION)
add_header_group(${PYWRAP}_HEADERS)
add_source_group(${PYWRAP}_SOURCES)

if(APPLE)
  # We need to change the extension for python bindings
  set_target_properties(${PYWRAP} PROPERTIES SUFFIX ".so")
endif(APPLE)

set_target_properties(${PYWRAP} PROPERTIES LIBRARY_OUTPUT_DIRECTORY
                                           ${PROJECT_NAME})

if(UNIX AND NOT APPLE)
  set_target_properties(${PYWRAP} PROPERTIES INSTALL_RPATH "\$ORIGIN/../../..")
endif()

install(
  TARGETS ${PYWRAP}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})

set(PYTHON_FILES __init__.py)

foreach(python ${PYTHON_FILES})
  python_install_on_site(${PROJECT_NAME} ${python})
endforeach(python)
