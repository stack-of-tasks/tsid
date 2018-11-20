# --- MACROS --- #
MACRO(SYMLINK_AND_INSTALL_HEADERS HEADERS SUBPATH)
  FOREACH(header ${HEADERS})
    GET_FILENAME_COMPONENT(headerName ${header} NAME)
    GET_FILENAME_COMPONENT(headerPath ${header} PATH)
    EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E ${LINK}
      ${CMAKE_CURRENT_SOURCE_DIR}/${header}
      ${${PROJECT_NAME}_BINARY_DIR}/include/${PROJECT_NAME}/${SUBPATH}/${header})

    INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/${header}
      DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/${SUBPATH}/${headerPath}
            PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)
  ENDFOREACH(header)
ENDMACRO(SYMLINK_AND_INSTALL_HEADERS HEADERS SUBPATH)

# --- PYTHON TARGET --- #
ADD_CUSTOM_TARGET(python)
SET_TARGET_PROPERTIES(python PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD True)

# --- DEPENDENCIES --- #
SET(PKG_CONFIG_PYWRAP_REQUIRES "eigenpy >= 1.4.0")
FOREACH(dep ${PKG_CONFIG_PYWRAP_REQUIRES})
  ADD_COMPILE_DEPENDENCY(${dep})
ENDFOREACH(dep ${PKG_CONFIG_PYWRAP_REQUIRES})

# --- LIBRARY --- #
SET(${PYWRAP}_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITELIB}/${PROJECT_NAME})

SET(${PROJECT_NAME}_PYTHON_HEADERS
  robots/robot-wrapper.hpp
  robots/expose-robots.hpp
  constraint/constraint-bound.hpp
  constraint/constraint-equality.hpp
  constraint/constraint-inequality.hpp
  constraint/expose-constraints.hpp
  contacts/expose-contact.hpp
  contacts/contact.hpp
  trajectories/expose-trajectories.hpp
  trajectories/trajectory-euclidian.hpp
  trajectories/trajectory-se3.hpp  
  trajectories/trajectory-base.hpp
  tasks/expose-tasks.hpp
  tasks/task-com-equality.hpp
  tasks/task-se3-equality.hpp
  tasks/task-joint-posture.hpp
  utils/container.hpp
  solvers/expose-solvers.hpp
  solvers/solver-HQP-eiquadprog.hpp
  solvers/HQPData.hpp
  solvers/HQPOutput.hpp
  formulations/formulation.hpp
  formulations/expose-formulations.hpp
  )

SET(${PROJECT_NAME}_PYTHON_SOURCES
  module.cpp
  robots/robot-wrapper.cpp  
  constraint/constraint-bound.cpp
  constraint/constraint-equality.cpp
  constraint/constraint-inequality.cpp
  contacts/contact.cpp
  trajectories/trajectory-euclidian.cpp
  trajectories/trajectory-se3.cpp
  trajectories/trajectory-base.cpp 
  tasks/task-com-equality.cpp
  tasks/task-se3-equality.cpp
  tasks/task-joint-posture.cpp
  solvers/solver-HQP-eiquadprog.cpp 
  solvers/HQPData.cpp
  solvers/HQPOutput.cpp
  formulations/formulation.cpp
  )


LIST(APPEND HEADERS ${${PROJECT_NAME}_PYTHON_HEADERS})
LIST(REMOVE_DUPLICATES HEADERS)

# Headers of the python binding
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/tsid/bindings/python")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/tsid/bindings/python/robots")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/tsid/bindings/python/constraint")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/tsid/bindings/python/contacts")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/tsid/bindings/python/trajectories")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/tsid/bindings/python/tasks")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/tsid/bindings/python/solvers")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/tsid/bindings/python/formulations")
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/include/tsid/bindings/python/utils")

SYMLINK_AND_INSTALL_HEADERS("${${PROJECT_NAME}_PYTHON_HEADERS}" "bindings/python")

# --- COMPILE WRAPPER
MAKE_DIRECTORY("${${PROJECT_NAME}_BINARY_DIR}/bindings/python/${PROJECT_NAME}")
SET(${PYWRAP}_SOURCES ${${PROJECT_NAME}_PYTHON_SOURCES})
SET(${PYWRAP}_HEADERS ${${PROJECT_NAME}_PYTHON_HEADERS})

ADD_LIBRARY(${PYWRAP} SHARED ${${PYWRAP}_SOURCES} ${${PYWRAP}_HEADERS})
ADD_DEPENDENCIES(python ${PYWRAP})

IF(BUILD_WITH_COMMIT_VERSION)
  TAG_LIBRARY_VERSION(${PYWRAP})
ENDIF(BUILD_WITH_COMMIT_VERSION)
ADD_HEADER_GROUP(${PYWRAP}_HEADERS)
ADD_SOURCE_GROUP(${PYWRAP}_SOURCES)
PKG_CONFIG_USE_DEPENDENCY(${PYWRAP} eigenpy)

TARGET_LINK_LIBRARIES(${PYWRAP} ${PROJECT_NAME})
TARGET_LINK_BOOST_PYTHON(${PYWRAP})

IF(APPLE)
  # We need to change the extension for python bindings
  SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES SUFFIX ".so")
ENDIF(APPLE)

SET_TARGET_PROPERTIES(${PYWRAP} PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bindings/python/${PROJECT_NAME}")

INSTALL(TARGETS ${PYWRAP} DESTINATION ${${PYWRAP}_INSTALL_DIR})

# --- INSTALL SCRIPTS 
SET(PYTHON_FILES
 __init__.py
  )

FOREACH(python ${PYTHON_FILES})
  INSTALL(FILES
    "${${PROJECT_NAME}_SOURCE_DIR}/bindings/python/${python}"
    DESTINATION ${${PYWRAP}_INSTALL_DIR})
ENDFOREACH(python)

# --- PACKAGING --- #

# Format string
SET(_PKG_CONFIG_PYWRAP_LIBDIR ${${PYWRAP}_INSTALL_DIR})
SET(_PKG_CONFIG_PYWRAP_BINDIR ${${PYWRAP}_INSTALL_DIR})
SET(_PKG_CONFIG_PYWRAP_CONFLICTS)
SET(_PKG_CONFIG_PYWRAP_REQUIRES "${PROJECT_NAME}")
FOREACH(dep ${PKG_CONFIG_PYWRAP_REQUIRES})
  SET(_PKG_CONFIG_PYWRAP_REQUIRES "${_PKG_CONFIG_PYWRAP_REQUIRES}, ${dep}")
ENDFOREACH(dep ${PKG_CONFIG_PYWRAP_REQUIRES})

SET(_PKG_CONFIG_PYWRAP_LIBS "-L\${libdir} -l${PYWRAP}")
IF(APPLE)
  SET(_PKG_CONFIG_PYWRAP_LIBS "${_PKG_CONFIG_PYWRAP_LIBS} -Wl,-undefined,dynamic_lookup,${Boost_${UPPERCOMPONENT}_LIBRARY}")
ELSE(APPLE)
  SET(_PKG_CONFIG_PYWRAP_LIBS "${_PKG_CONFIG_PYWRAP_LIBS} ${LIBINCL_KW}boost_python")
ENDIF(APPLE)

SET(_PKG_CONFIG_PYWRAP_CFLAGS "-I\${includedir} -I/usr/local/include")
SET(_PKG_CONFIG_PYWRAP_CFLAGS "${_PKG_CONFIG_PYWRAP_CFLAGS} -I${PYTHON_INCLUDE_DIRS}")
FOREACH(cflags ${CFLAGS_DEPENDENCIES})
  SET(_PKG_CONFIG_PYWRAP_CFLAGS "${_PKG_CONFIG_PYWRAP_CFLAGS} ${cflags}")
ENDFOREACH(cflags ${CFLAGS_DEPENDENCIES})

CONFIGURE_FILE(
   "${CMAKE_CURRENT_SOURCE_DIR}/tsidpy.pc.cmake"
   "${CMAKE_CURRENT_BINARY_DIR}/tsidpy.pc")

INSTALL(
    FILES "${CMAKE_CURRENT_BINARY_DIR}/tsidpy.pc"
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE)

