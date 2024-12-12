# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "swarm_aggregation: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iswarm_aggregation:/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Inav_msgs:/opt/ros/noetic/share/nav_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/noetic/share/actionlib_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(swarm_aggregation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/bot.msg" NAME_WE)
add_custom_target(_swarm_aggregation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "swarm_aggregation" "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/bot.msg" ""
)

get_filename_component(_filename "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/botPose.msg" NAME_WE)
add_custom_target(_swarm_aggregation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "swarm_aggregation" "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/botPose.msg" "nav_msgs/Odometry:geometry_msgs/Twist:geometry_msgs/Point:geometry_msgs/Quaternion:geometry_msgs/TwistWithCovariance:geometry_msgs/Pose:geometry_msgs/PoseWithCovariance:geometry_msgs/Vector3:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(swarm_aggregation
  "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/bot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_aggregation
)
_generate_msg_cpp(swarm_aggregation
  "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/botPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_aggregation
)

### Generating Services

### Generating Module File
_generate_module_cpp(swarm_aggregation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_aggregation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(swarm_aggregation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(swarm_aggregation_generate_messages swarm_aggregation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/bot.msg" NAME_WE)
add_dependencies(swarm_aggregation_generate_messages_cpp _swarm_aggregation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/botPose.msg" NAME_WE)
add_dependencies(swarm_aggregation_generate_messages_cpp _swarm_aggregation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_aggregation_gencpp)
add_dependencies(swarm_aggregation_gencpp swarm_aggregation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_aggregation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(swarm_aggregation
  "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/bot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_aggregation
)
_generate_msg_eus(swarm_aggregation
  "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/botPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_aggregation
)

### Generating Services

### Generating Module File
_generate_module_eus(swarm_aggregation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_aggregation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(swarm_aggregation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(swarm_aggregation_generate_messages swarm_aggregation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/bot.msg" NAME_WE)
add_dependencies(swarm_aggregation_generate_messages_eus _swarm_aggregation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/botPose.msg" NAME_WE)
add_dependencies(swarm_aggregation_generate_messages_eus _swarm_aggregation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_aggregation_geneus)
add_dependencies(swarm_aggregation_geneus swarm_aggregation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_aggregation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(swarm_aggregation
  "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/bot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_aggregation
)
_generate_msg_lisp(swarm_aggregation
  "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/botPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_aggregation
)

### Generating Services

### Generating Module File
_generate_module_lisp(swarm_aggregation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_aggregation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(swarm_aggregation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(swarm_aggregation_generate_messages swarm_aggregation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/bot.msg" NAME_WE)
add_dependencies(swarm_aggregation_generate_messages_lisp _swarm_aggregation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/botPose.msg" NAME_WE)
add_dependencies(swarm_aggregation_generate_messages_lisp _swarm_aggregation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_aggregation_genlisp)
add_dependencies(swarm_aggregation_genlisp swarm_aggregation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_aggregation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(swarm_aggregation
  "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/bot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_aggregation
)
_generate_msg_nodejs(swarm_aggregation
  "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/botPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_aggregation
)

### Generating Services

### Generating Module File
_generate_module_nodejs(swarm_aggregation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_aggregation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(swarm_aggregation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(swarm_aggregation_generate_messages swarm_aggregation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/bot.msg" NAME_WE)
add_dependencies(swarm_aggregation_generate_messages_nodejs _swarm_aggregation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/botPose.msg" NAME_WE)
add_dependencies(swarm_aggregation_generate_messages_nodejs _swarm_aggregation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_aggregation_gennodejs)
add_dependencies(swarm_aggregation_gennodejs swarm_aggregation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_aggregation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(swarm_aggregation
  "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/bot.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_aggregation
)
_generate_msg_py(swarm_aggregation
  "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/botPose.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/nav_msgs/cmake/../msg/Odometry.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/TwistWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/PoseWithCovariance.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_aggregation
)

### Generating Services

### Generating Module File
_generate_module_py(swarm_aggregation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_aggregation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(swarm_aggregation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(swarm_aggregation_generate_messages swarm_aggregation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/bot.msg" NAME_WE)
add_dependencies(swarm_aggregation_generate_messages_py _swarm_aggregation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/arms/rohand_ws/swarm_ws/src/swarm_aggregation/msg/botPose.msg" NAME_WE)
add_dependencies(swarm_aggregation_generate_messages_py _swarm_aggregation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(swarm_aggregation_genpy)
add_dependencies(swarm_aggregation_genpy swarm_aggregation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS swarm_aggregation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_aggregation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/swarm_aggregation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(swarm_aggregation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET nav_msgs_generate_messages_cpp)
  add_dependencies(swarm_aggregation_generate_messages_cpp nav_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_aggregation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/swarm_aggregation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(swarm_aggregation_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET nav_msgs_generate_messages_eus)
  add_dependencies(swarm_aggregation_generate_messages_eus nav_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_aggregation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/swarm_aggregation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(swarm_aggregation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET nav_msgs_generate_messages_lisp)
  add_dependencies(swarm_aggregation_generate_messages_lisp nav_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_aggregation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/swarm_aggregation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(swarm_aggregation_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET nav_msgs_generate_messages_nodejs)
  add_dependencies(swarm_aggregation_generate_messages_nodejs nav_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_aggregation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_aggregation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/swarm_aggregation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(swarm_aggregation_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET nav_msgs_generate_messages_py)
  add_dependencies(swarm_aggregation_generate_messages_py nav_msgs_generate_messages_py)
endif()
