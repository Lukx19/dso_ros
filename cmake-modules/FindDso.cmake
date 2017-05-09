# Locate the DSO libraries
# A direct sparse odometry.
#
# This module defines
# DSO_FOUND, if false, do not try to link against dso
# DSO_LIBRARIES, path to the dso libraries
# DSO_INCLUDE_DIRS, where to find the dso header files
#
# Lukas Jelinek <lukx19@gmail.com>

IF(UNIX)

  IF(DSO_INCLUDE_DIRS AND DSO_LIBRARIES)
    # in cache already
    SET(DSO_FIND_QUIETLY TRUE)
  ENDIF(DSO_INCLUDE_DIRS AND DSO_LIBRARIES)

  MESSAGE(STATUS "Searching for DSO ...")
  FIND_PATH(DSO_INCLUDE_CORE
    NAMES IOWrapper FullSystem
    PATHS /usr/local /usr $ENV{DSO_PATH}
    PATH_SUFFIXES src dso/src)

  FIND_PATH(DSO_INCLUDE_SOPHUS
    NAMES sophus
    PATHS /usr/local /usr $ENV{DSO_PATH}
    PATH_SUFFIXES thirdparty/Sophus dso/thirdparty/Sophus)

  SET(DSO_INCLUDE_DIRS ${DSO_INCLUDE_CORE} ${DSO_INCLUDE_SOPHUS})

  IF (DSO_INCLUDE_DIRS)
    MESSAGE(STATUS "Found DSO headers in: ${DSO_INCLUDE_DIRS}")
  ENDIF (DSO_INCLUDE_DIRS)

  FIND_LIBRARY(DSO_CORE_LIB
    NAMES dso
    PATHS /usr/local /usr $ENV{DSO_PATH}
    PATH_SUFFIXES build/lib dso/build/lib)
  SET(DSO_LIBRARIES ${DSO_CORE_LIB})

  IF(DSO_LIBRARIES AND DSO_INCLUDE_DIRS)
    SET(DSO_FOUND "YES")
    IF(NOT DSO_FIND_QUIETLY)
      MESSAGE(STATUS "Found libDSO: ${DSO_LIBRARIES}")
    ENDIF(NOT DSO_FIND_QUIETLY)
  ELSE(DSO_LIBRARIES AND DSO_INCLUDE_DIRS)
    IF(NOT DSO_LIBRARIES)
      IF(DSO_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find libDSO!")
      ENDIF(DSO_FIND_REQUIRED)
    ENDIF(NOT DSO_LIBRARIES)

    IF(NOT DSO_INCLUDE_DIRS)
      IF(DSO_FIND_REQUIRED)
        message(FATAL_ERROR "Could not find DSO include directory!")
      ENDIF(DSO_FIND_REQUIRED)
    ENDIF(NOT DSO_INCLUDE_DIRS)
  ENDIF(DSO_LIBRARIES AND DSO_INCLUDE_DIRS)

ENDIF(UNIX)
