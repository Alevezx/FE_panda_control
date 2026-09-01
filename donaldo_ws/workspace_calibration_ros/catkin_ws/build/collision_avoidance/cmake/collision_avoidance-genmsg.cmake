# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "collision_avoidance: 0 messages, 2 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(collision_avoidance_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv" NAME_WE)
add_custom_target(_collision_avoidance_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collision_avoidance" "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv" ""
)

get_filename_component(_filename "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv" NAME_WE)
add_custom_target(_collision_avoidance_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "collision_avoidance" "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(collision_avoidance
  "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collision_avoidance
)
_generate_srv_cpp(collision_avoidance
  "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collision_avoidance
)

### Generating Module File
_generate_module_cpp(collision_avoidance
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collision_avoidance
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(collision_avoidance_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(collision_avoidance_generate_messages collision_avoidance_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv" NAME_WE)
add_dependencies(collision_avoidance_generate_messages_cpp _collision_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv" NAME_WE)
add_dependencies(collision_avoidance_generate_messages_cpp _collision_avoidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collision_avoidance_gencpp)
add_dependencies(collision_avoidance_gencpp collision_avoidance_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collision_avoidance_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(collision_avoidance
  "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collision_avoidance
)
_generate_srv_eus(collision_avoidance
  "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collision_avoidance
)

### Generating Module File
_generate_module_eus(collision_avoidance
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collision_avoidance
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(collision_avoidance_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(collision_avoidance_generate_messages collision_avoidance_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv" NAME_WE)
add_dependencies(collision_avoidance_generate_messages_eus _collision_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv" NAME_WE)
add_dependencies(collision_avoidance_generate_messages_eus _collision_avoidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collision_avoidance_geneus)
add_dependencies(collision_avoidance_geneus collision_avoidance_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collision_avoidance_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(collision_avoidance
  "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collision_avoidance
)
_generate_srv_lisp(collision_avoidance
  "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collision_avoidance
)

### Generating Module File
_generate_module_lisp(collision_avoidance
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collision_avoidance
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(collision_avoidance_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(collision_avoidance_generate_messages collision_avoidance_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv" NAME_WE)
add_dependencies(collision_avoidance_generate_messages_lisp _collision_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv" NAME_WE)
add_dependencies(collision_avoidance_generate_messages_lisp _collision_avoidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collision_avoidance_genlisp)
add_dependencies(collision_avoidance_genlisp collision_avoidance_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collision_avoidance_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(collision_avoidance
  "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collision_avoidance
)
_generate_srv_nodejs(collision_avoidance
  "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collision_avoidance
)

### Generating Module File
_generate_module_nodejs(collision_avoidance
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collision_avoidance
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(collision_avoidance_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(collision_avoidance_generate_messages collision_avoidance_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv" NAME_WE)
add_dependencies(collision_avoidance_generate_messages_nodejs _collision_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv" NAME_WE)
add_dependencies(collision_avoidance_generate_messages_nodejs _collision_avoidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collision_avoidance_gennodejs)
add_dependencies(collision_avoidance_gennodejs collision_avoidance_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collision_avoidance_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(collision_avoidance
  "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_avoidance
)
_generate_srv_py(collision_avoidance
  "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_avoidance
)

### Generating Module File
_generate_module_py(collision_avoidance
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_avoidance
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(collision_avoidance_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(collision_avoidance_generate_messages collision_avoidance_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv" NAME_WE)
add_dependencies(collision_avoidance_generate_messages_py _collision_avoidance_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv" NAME_WE)
add_dependencies(collision_avoidance_generate_messages_py _collision_avoidance_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(collision_avoidance_genpy)
add_dependencies(collision_avoidance_genpy collision_avoidance_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS collision_avoidance_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collision_avoidance)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/collision_avoidance
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(collision_avoidance_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collision_avoidance)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/collision_avoidance
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(collision_avoidance_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collision_avoidance)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/collision_avoidance
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(collision_avoidance_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collision_avoidance)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/collision_avoidance
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(collision_avoidance_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_avoidance)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_avoidance\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/collision_avoidance
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(collision_avoidance_generate_messages_py std_msgs_generate_messages_py)
endif()
