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

# --- LIBRARY --- #
FILE(GLOB ${PYWRAP}_SOURCES
  *.cpp
  constraint/*.cpp
  contacts/*.cpp
  formulations/*.cpp
  robots/*.cpp
  solvers/*.cpp
  tasks/*.cpp
  trajectories/*.cpp
  utils/*.cpp
  )

ADD_LIBRARY(${PYWRAP} SHARED ${${PYWRAP}_SOURCES} ${${PYWRAP}_HEADERS})
TARGET_LINK_LIBRARIES(${PYWRAP} PUBLIC ${PROJECT_NAME} eigenpy::eigenpy)
TARGET_LINK_BOOST_PYTHON(${PYWRAP} PRIVATE)

IF(BUILD_WITH_COMMIT_VERSION)
  TAG_LIBRARY_VERSION(${PYWRAP})
ENDIF(BUILD_WITH_COMMIT_VERSION)
ADD_HEADER_GROUP(${PYWRAP}_HEADERS)
ADD_SOURCE_GROUP(${PYWRAP}_SOURCES)

IF(APPLE)
  # We need to change the extension for python bindings
  SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES SUFFIX ".so")
ENDIF(APPLE)

SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_NAME})

INSTALL(TARGETS ${PYWRAP} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION ${PYTHON_SITELIB}/${PROJECT_NAME})

SET(PYTHON_FILES
  __init__.py
  )

FOREACH(python ${PYTHON_FILES})
  PYTHON_INSTALL_ON_SITE(${PROJECT_NAME} ${python})
ENDFOREACH(python)
