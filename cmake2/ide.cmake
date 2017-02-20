# Copyright (C) 2008-2016 LAAS-CNRS, JRL AIST-CNRS.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <http://www.gnu.org/licenses/>.

MACRO(ADD_GROUP GROUP_NAME FILENAMES)
  FOREACH(filename ${${FILENAMES}})
    GET_FILENAME_COMPONENT(filenamePath ${filename} PATH)
    IF(NOT (filenamePath STREQUAL ""))
      STRING(REGEX REPLACE "${CMAKE_SOURCE_DIR}/include" "" filenamePath ${filenamePath}) 
      STRING(REGEX REPLACE "${CMAKE_SOURCE_DIR}" "" filenamePath ${filenamePath}) 
      STRING(REGEX REPLACE "/" "\\\\" filenamePath ${filenamePath}) 
      SOURCE_GROUP("${GROUP_NAME}\\${filenamePath}" FILES ${filename})
    ELSE()
      SOURCE_GROUP("${GROUP_NAME}" FILES ${filename})
    ENDIF()
  ENDFOREACH()
ENDMACRO(ADD_GROUP) 

MACRO(ADD_HEADER_GROUP FILENAMES)
  ADD_GROUP("Header Files" ${FILENAMES})
ENDMACRO(ADD_HEADER_GROUP FILENAMES)

MACRO(ADD_SOURCE_GROUP FILENAMES)
  ADD_GROUP("Source Files" ${FILENAMES})
ENDMACRO(ADD_SOURCE_GROUP FILENAMES)

