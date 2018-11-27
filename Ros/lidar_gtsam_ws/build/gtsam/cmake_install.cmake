# Install script for directory: /home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/gtsam

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam" TYPE FILE FILES
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/gtsam/precompiled_header.h"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10/gtsam/global_includes.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/gtsam" TYPE FILE FILES
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/config.h"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/dllexport.h"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so.4.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so.4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/libgtsam.so.4.0.0"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/libgtsam.so.4"
    "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/libgtsam.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so.4.0.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so.4"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libgtsam.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHANGE
           FILE "${file}"
           OLD_RPATH "/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/3rdparty/metis/libmetis:"
           NEW_RPATH "")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/3rdparty/cmake_install.cmake")
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/base/cmake_install.cmake")
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/geometry/cmake_install.cmake")
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/inference/cmake_install.cmake")
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/symbolic/cmake_install.cmake")
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/discrete/cmake_install.cmake")
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/linear/cmake_install.cmake")
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/nonlinear/cmake_install.cmake")
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/sam/cmake_install.cmake")
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/sfm/cmake_install.cmake")
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/slam/cmake_install.cmake")
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/smart/cmake_install.cmake")
  include("/home/rip-acer-vn7-591g-1/lidar_gtsam_ws/build/gtsam/navigation/cmake_install.cmake")

endif()

