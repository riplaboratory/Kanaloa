# Install script for directory: /home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake

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
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/GTSAMCMakeTools/GTSAMCMakeToolsConfig.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/Config.cmake.in;/usr/local/lib/cmake/GTSAMCMakeTools/dllexport.h.in;/usr/local/lib/cmake/GTSAMCMakeTools/GtsamBuildTypes.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/GtsamMakeConfigFile.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/GtsamMatlabWrap.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/GtsamPythonWrap.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/GtsamCythonWrap.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/GtsamTesting.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/FindCython.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/FindNumPy.cmake;/usr/local/lib/cmake/GTSAMCMakeTools/README.html")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib/cmake/GTSAMCMakeTools" TYPE FILE FILES
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake/GTSAMCMakeToolsConfig.cmake"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake/Config.cmake.in"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake/dllexport.h.in"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake/GtsamBuildTypes.cmake"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake/GtsamMakeConfigFile.cmake"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake/GtsamMatlabWrap.cmake"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake/GtsamPythonWrap.cmake"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake/GtsamCythonWrap.cmake"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake/GtsamTesting.cmake"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake/FindCython.cmake"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake/FindNumPy.cmake"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/cmake/README.html"
    )
endif()

