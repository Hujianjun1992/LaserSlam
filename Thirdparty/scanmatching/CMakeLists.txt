cmake_minimum_required(VERSION 2.8.3)
project(scanmatching)


option(STRICT_CENSI_COVARIANCE "Use strict censi covariances" OFF)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(src/ANN/include)


IF(EXISTS ${PROJECT_CMAKE}/GslConfig.cmake)
  INCLUDE(${PROJECT_CMAKE}/GslConfig.cmake)
ENDIF(EXISTS ${PROJECT_CMAKE}/GslConfig.cmake)

IF(Gsl_INCLUDE_DIRS)

  FIND_PATH(GSL_INCLUDE_DIR gsl/gsl_version.h ${Gsl_INCLUDE_DIRS})
  FIND_LIBRARY(GSL_LIBRARY gsl ${Gsl_LIBRARY_DIRS})

ELSE(Gsl_INCLUDE_DIRS)

  FIND_LIBRARY(GSL_LIBRARY gsl $ENV{GSL_HOME}/lib ${QMC_LIBRARY_PATHS})
  FIND_LIBRARY(GSLCBLAS_LIBRARY gslcblas  $ENV{GSL_HOME}/lib ${QMC_LIBRARY_PATHS})
  FIND_PATH(GSL_INCLUDE_DIR gsl/gsl_version.h $ENV{GSL_HOME}/include ${QMC_INCLUDE_PATHS} )

ENDIF(Gsl_INCLUDE_DIRS)

IF(GSL_INCLUDE_DIR AND GSL_LIBRARY)
  SET(GSL_FOUND 1 CACHE BOOL "Found gsl library")
  SET(GSL_LIBRARY ${GSL_LIBRARY} ${GSLCBLAS_LIBRARY})
ELSE(GSL_INCLUDE_DIR AND GSL_LIBRARY)
  SET(GSL_FOUND 0 CACHE BOOL "Not fount gsl library")
ENDIF(GSL_INCLUDE_DIR AND GSL_LIBRARY)

MARK_AS_ADVANCED(
  GSL_INCLUDE_DIR 
  GSL_LIBRARY 
  GSL_FOUND
  )


# Look for GSL library - END

if(STRICT_CENSI_COVARIANCE)
	add_definitions(-DSTRICT_CENSI_COVARIANCE)
endif(STRICT_CENSI_COVARIANCE)


add_library(scanmatching SHARED
	Config.cpp
	src/ClassicICP.cpp
	src/ICP.cpp
	src/MetricICP.cpp
	src/NearestNeighboursANN.cpp
	src/NearestNeighboursON2.cpp
	src/PointToLineICP.cpp
	src/Rototranslation2D.cpp
	src/Scan.cpp
	src/ANN/src/ANN.cpp
	src/ANN/src/bd_fix_rad_search.cpp
	src/ANN/src/bd_pr_search.cpp
	src/ANN/src/bd_search.cpp
	src/ANN/src/bd_tree.cpp
	src/ANN/src/brute.cpp
	src/ANN/src/kd_dump.cpp
	src/ANN/src/kd_fix_rad_search.cpp
	src/ANN/src/kd_pr_search.cpp
	src/ANN/src/kd_search.cpp
	src/ANN/src/kd_split.cpp
	src/ANN/src/kd_tree.cpp
	src/ANN/src/kd_util.cpp
	src/ANN/src/perf.cpp
	src/gpc/gpc.c
	src/gpc/gpc_utils.c)


target_link_libraries(scanmatching gsl gslcblas)
