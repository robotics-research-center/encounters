# Install script for directory: /home/gunshi/Downloads/MultiRobot/3dmerge/g2o/types/sclam2d

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/libg2o_types_sclam2d.a")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib" TYPE STATIC_LIBRARY FILES "/home/gunshi/Downloads/MultiRobot/3dmerge/lib/libg2o_types_sclam2d.a")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/g2o/types/sclam2d/g2o_types_sclam2d_api.h;/usr/local/include/g2o/types/sclam2d/edge_se2_sensor_calib.h;/usr/local/include/g2o/types/sclam2d/vertex_odom_differential_params.h;/usr/local/include/g2o/types/sclam2d/edge_se2_odom_differential_calib.h;/usr/local/include/g2o/types/sclam2d/types_sclam2d.h;/usr/local/include/g2o/types/sclam2d/odometry_measurement.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/include/g2o/types/sclam2d" TYPE FILE FILES
    "/home/gunshi/Downloads/MultiRobot/3dmerge/g2o/types/sclam2d/g2o_types_sclam2d_api.h"
    "/home/gunshi/Downloads/MultiRobot/3dmerge/g2o/types/sclam2d/edge_se2_sensor_calib.h"
    "/home/gunshi/Downloads/MultiRobot/3dmerge/g2o/types/sclam2d/vertex_odom_differential_params.h"
    "/home/gunshi/Downloads/MultiRobot/3dmerge/g2o/types/sclam2d/edge_se2_odom_differential_calib.h"
    "/home/gunshi/Downloads/MultiRobot/3dmerge/g2o/types/sclam2d/types_sclam2d.h"
    "/home/gunshi/Downloads/MultiRobot/3dmerge/g2o/types/sclam2d/odometry_measurement.h"
    )
endif()

