# Install script for directory: /home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/gtsam/sam

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
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam/sam" TYPE FILE FILES
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/gtsam/sam/BearingFactor.h"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/gtsam/sam/BearingRangeFactor.h"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/gtsam/sam/RangeFactor.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/sam/tests/cmake_install.cmake")

endif()

