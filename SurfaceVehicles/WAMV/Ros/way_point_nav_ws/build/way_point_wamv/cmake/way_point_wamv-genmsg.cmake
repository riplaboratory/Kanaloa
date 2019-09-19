# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "way_point_wamv: 0 messages, 1 services")

set(MSG_I_FLAGS "-Isensor_msgs:/opt/ros/melodic/share/sensor_msgs/cmake/../msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(way_point_wamv_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rip-acer-2/Workspaces/way_point_nav_ws/src/way_point_wamv/srv/add_way_point.srv" NAME_WE)
add_custom_target(_way_point_wamv_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "way_point_wamv" "/home/rip-acer-2/Workspaces/way_point_nav_ws/src/way_point_wamv/srv/add_way_point.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(way_point_wamv
  "/home/rip-acer-2/Workspaces/way_point_nav_ws/src/way_point_wamv/srv/add_way_point.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/way_point_wamv
)

### Generating Module File
_generate_module_cpp(way_point_wamv
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/way_point_wamv
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(way_point_wamv_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(way_point_wamv_generate_messages way_point_wamv_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rip-acer-2/Workspaces/way_point_nav_ws/src/way_point_wamv/srv/add_way_point.srv" NAME_WE)
add_dependencies(way_point_wamv_generate_messages_cpp _way_point_wamv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(way_point_wamv_gencpp)
add_dependencies(way_point_wamv_gencpp way_point_wamv_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS way_point_wamv_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(way_point_wamv
  "/home/rip-acer-2/Workspaces/way_point_nav_ws/src/way_point_wamv/srv/add_way_point.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/way_point_wamv
)

### Generating Module File
_generate_module_eus(way_point_wamv
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/way_point_wamv
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(way_point_wamv_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(way_point_wamv_generate_messages way_point_wamv_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rip-acer-2/Workspaces/way_point_nav_ws/src/way_point_wamv/srv/add_way_point.srv" NAME_WE)
add_dependencies(way_point_wamv_generate_messages_eus _way_point_wamv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(way_point_wamv_geneus)
add_dependencies(way_point_wamv_geneus way_point_wamv_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS way_point_wamv_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(way_point_wamv
  "/home/rip-acer-2/Workspaces/way_point_nav_ws/src/way_point_wamv/srv/add_way_point.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/way_point_wamv
)

### Generating Module File
_generate_module_lisp(way_point_wamv
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/way_point_wamv
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(way_point_wamv_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(way_point_wamv_generate_messages way_point_wamv_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rip-acer-2/Workspaces/way_point_nav_ws/src/way_point_wamv/srv/add_way_point.srv" NAME_WE)
add_dependencies(way_point_wamv_generate_messages_lisp _way_point_wamv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(way_point_wamv_genlisp)
add_dependencies(way_point_wamv_genlisp way_point_wamv_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS way_point_wamv_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(way_point_wamv
  "/home/rip-acer-2/Workspaces/way_point_nav_ws/src/way_point_wamv/srv/add_way_point.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/way_point_wamv
)

### Generating Module File
_generate_module_nodejs(way_point_wamv
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/way_point_wamv
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(way_point_wamv_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(way_point_wamv_generate_messages way_point_wamv_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rip-acer-2/Workspaces/way_point_nav_ws/src/way_point_wamv/srv/add_way_point.srv" NAME_WE)
add_dependencies(way_point_wamv_generate_messages_nodejs _way_point_wamv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(way_point_wamv_gennodejs)
add_dependencies(way_point_wamv_gennodejs way_point_wamv_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS way_point_wamv_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(way_point_wamv
  "/home/rip-acer-2/Workspaces/way_point_nav_ws/src/way_point_wamv/srv/add_way_point.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/way_point_wamv
)

### Generating Module File
_generate_module_py(way_point_wamv
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/way_point_wamv
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(way_point_wamv_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(way_point_wamv_generate_messages way_point_wamv_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rip-acer-2/Workspaces/way_point_nav_ws/src/way_point_wamv/srv/add_way_point.srv" NAME_WE)
add_dependencies(way_point_wamv_generate_messages_py _way_point_wamv_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(way_point_wamv_genpy)
add_dependencies(way_point_wamv_genpy way_point_wamv_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS way_point_wamv_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/way_point_wamv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/way_point_wamv
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(way_point_wamv_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(way_point_wamv_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/way_point_wamv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/way_point_wamv
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(way_point_wamv_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(way_point_wamv_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/way_point_wamv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/way_point_wamv
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(way_point_wamv_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(way_point_wamv_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/way_point_wamv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/way_point_wamv
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(way_point_wamv_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(way_point_wamv_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/way_point_wamv)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/way_point_wamv\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/way_point_wamv
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(way_point_wamv_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(way_point_wamv_generate_messages_py std_msgs_generate_messages_py)
endif()
