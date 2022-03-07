# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "LKOpticalFlow_msgs: 1 messages, 0 services")

set(MSG_I_FLAGS "-ILKOpticalFlow_msgs:/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(LKOpticalFlow_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg/Points.msg" NAME_WE)
add_custom_target(_LKOpticalFlow_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "LKOpticalFlow_msgs" "/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg/Points.msg" "std_msgs/MultiArrayLayout:std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(LKOpticalFlow_msgs
  "/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg/Points.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/LKOpticalFlow_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(LKOpticalFlow_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/LKOpticalFlow_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(LKOpticalFlow_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(LKOpticalFlow_msgs_generate_messages LKOpticalFlow_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg/Points.msg" NAME_WE)
add_dependencies(LKOpticalFlow_msgs_generate_messages_cpp _LKOpticalFlow_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(LKOpticalFlow_msgs_gencpp)
add_dependencies(LKOpticalFlow_msgs_gencpp LKOpticalFlow_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS LKOpticalFlow_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(LKOpticalFlow_msgs
  "/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg/Points.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/LKOpticalFlow_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(LKOpticalFlow_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/LKOpticalFlow_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(LKOpticalFlow_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(LKOpticalFlow_msgs_generate_messages LKOpticalFlow_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg/Points.msg" NAME_WE)
add_dependencies(LKOpticalFlow_msgs_generate_messages_eus _LKOpticalFlow_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(LKOpticalFlow_msgs_geneus)
add_dependencies(LKOpticalFlow_msgs_geneus LKOpticalFlow_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS LKOpticalFlow_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(LKOpticalFlow_msgs
  "/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg/Points.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/LKOpticalFlow_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(LKOpticalFlow_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/LKOpticalFlow_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(LKOpticalFlow_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(LKOpticalFlow_msgs_generate_messages LKOpticalFlow_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg/Points.msg" NAME_WE)
add_dependencies(LKOpticalFlow_msgs_generate_messages_lisp _LKOpticalFlow_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(LKOpticalFlow_msgs_genlisp)
add_dependencies(LKOpticalFlow_msgs_genlisp LKOpticalFlow_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS LKOpticalFlow_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(LKOpticalFlow_msgs
  "/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg/Points.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/LKOpticalFlow_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(LKOpticalFlow_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/LKOpticalFlow_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(LKOpticalFlow_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(LKOpticalFlow_msgs_generate_messages LKOpticalFlow_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg/Points.msg" NAME_WE)
add_dependencies(LKOpticalFlow_msgs_generate_messages_nodejs _LKOpticalFlow_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(LKOpticalFlow_msgs_gennodejs)
add_dependencies(LKOpticalFlow_msgs_gennodejs LKOpticalFlow_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS LKOpticalFlow_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(LKOpticalFlow_msgs
  "/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg/Points.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/LKOpticalFlow_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(LKOpticalFlow_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/LKOpticalFlow_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(LKOpticalFlow_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(LKOpticalFlow_msgs_generate_messages LKOpticalFlow_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hao/motion_tracking_design/src/LKOpticalFlow_msgs/msg/Points.msg" NAME_WE)
add_dependencies(LKOpticalFlow_msgs_generate_messages_py _LKOpticalFlow_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(LKOpticalFlow_msgs_genpy)
add_dependencies(LKOpticalFlow_msgs_genpy LKOpticalFlow_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS LKOpticalFlow_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/LKOpticalFlow_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/LKOpticalFlow_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(LKOpticalFlow_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/LKOpticalFlow_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/LKOpticalFlow_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(LKOpticalFlow_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/LKOpticalFlow_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/LKOpticalFlow_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(LKOpticalFlow_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/LKOpticalFlow_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/LKOpticalFlow_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(LKOpticalFlow_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/LKOpticalFlow_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/LKOpticalFlow_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/LKOpticalFlow_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(LKOpticalFlow_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
