# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robotx_gazebo: 1 messages, 0 services")

set(MSG_I_FLAGS "-Irobotx_gazebo:/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Inav_msgs:/opt/ros/kinetic/share/nav_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robotx_gazebo_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg/UsvDrive.msg" NAME_WE)
add_custom_target(_robotx_gazebo_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robotx_gazebo" "/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg/UsvDrive.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robotx_gazebo
  "/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg/UsvDrive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotx_gazebo
)

### Generating Services

### Generating Module File
_generate_module_cpp(robotx_gazebo
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotx_gazebo
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robotx_gazebo_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robotx_gazebo_generate_messages robotx_gazebo_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg/UsvDrive.msg" NAME_WE)
add_dependencies(robotx_gazebo_generate_messages_cpp _robotx_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotx_gazebo_gencpp)
add_dependencies(robotx_gazebo_gencpp robotx_gazebo_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotx_gazebo_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robotx_gazebo
  "/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg/UsvDrive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotx_gazebo
)

### Generating Services

### Generating Module File
_generate_module_eus(robotx_gazebo
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotx_gazebo
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robotx_gazebo_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robotx_gazebo_generate_messages robotx_gazebo_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg/UsvDrive.msg" NAME_WE)
add_dependencies(robotx_gazebo_generate_messages_eus _robotx_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotx_gazebo_geneus)
add_dependencies(robotx_gazebo_geneus robotx_gazebo_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotx_gazebo_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robotx_gazebo
  "/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg/UsvDrive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotx_gazebo
)

### Generating Services

### Generating Module File
_generate_module_lisp(robotx_gazebo
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotx_gazebo
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robotx_gazebo_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robotx_gazebo_generate_messages robotx_gazebo_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg/UsvDrive.msg" NAME_WE)
add_dependencies(robotx_gazebo_generate_messages_lisp _robotx_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotx_gazebo_genlisp)
add_dependencies(robotx_gazebo_genlisp robotx_gazebo_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotx_gazebo_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robotx_gazebo
  "/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg/UsvDrive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotx_gazebo
)

### Generating Services

### Generating Module File
_generate_module_nodejs(robotx_gazebo
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotx_gazebo
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robotx_gazebo_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robotx_gazebo_generate_messages robotx_gazebo_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg/UsvDrive.msg" NAME_WE)
add_dependencies(robotx_gazebo_generate_messages_nodejs _robotx_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotx_gazebo_gennodejs)
add_dependencies(robotx_gazebo_gennodejs robotx_gazebo_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotx_gazebo_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robotx_gazebo
  "/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg/UsvDrive.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotx_gazebo
)

### Generating Services

### Generating Module File
_generate_module_py(robotx_gazebo
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotx_gazebo
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robotx_gazebo_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robotx_gazebo_generate_messages robotx_gazebo_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/rip-acer-vn7-591g-1/vmrc_ws/src/robotx_gazebo/msg/UsvDrive.msg" NAME_WE)
add_dependencies(robotx_gazebo_generate_messages_py _robotx_gazebo_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robotx_gazebo_genpy)
add_dependencies(robotx_gazebo_genpy robotx_gazebo_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robotx_gazebo_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotx_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robotx_gazebo
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(robotx_gazebo_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(robotx_gazebo_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robotx_gazebo_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotx_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robotx_gazebo
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(robotx_gazebo_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(robotx_gazebo_generate_messages_eus nav_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robotx_gazebo_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotx_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robotx_gazebo
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(robotx_gazebo_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(robotx_gazebo_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robotx_gazebo_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotx_gazebo)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robotx_gazebo
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(robotx_gazebo_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(robotx_gazebo_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robotx_gazebo_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotx_gazebo)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotx_gazebo\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robotx_gazebo
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(robotx_gazebo_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(robotx_gazebo_generate_messages_py nav_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robotx_gazebo_generate_messages_py std_msgs_generate_messages_py)
endif()
