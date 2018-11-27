# - Config file for GTSAM
# It defines the following variables
#  GTSAM_INCLUDE_DIR - include directories for GTSAM
 
# Compute paths
get_filename_component(OUR_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
if(EXISTS "${OUR_CMAKE_DIR}/CMakeCache.txt")
  # In build tree
  set(GTSAM_INCLUDE_DIR /home/rip-acer-vn7-591g-1/lidar_gtsam_ws/gtborg-gtsam-6f8bfe0f0a10 CACHE PATH "GTSAM include directory")
else()
  # Find installed library
  set(GTSAM_INCLUDE_DIR "${OUR_CMAKE_DIR}/../../../include" CACHE PATH "GTSAM include directory")
endif()
  
# Load exports
include(${OUR_CMAKE_DIR}/GTSAM-exports.cmake)

# Load project-specific flags, if present
if(EXISTS "${OUR_CMAKE_DIR}/gtsam_extra.cmake")
	include("${OUR_CMAKE_DIR}/gtsam_extra.cmake")
endif()

message(STATUS "GTSAM include directory:  ${GTSAM_INCLUDE_DIR}")
