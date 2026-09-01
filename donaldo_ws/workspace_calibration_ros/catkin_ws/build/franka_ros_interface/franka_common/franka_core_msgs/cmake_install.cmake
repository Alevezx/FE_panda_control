# Install script for directory: /home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/franka_ros_interface/franka_common/franka_core_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/install")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_core_msgs/msg" TYPE FILE FILES
    "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/franka_ros_interface/franka_common/franka_core_msgs/msg/JointCommand.msg"
    "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/franka_ros_interface/franka_common/franka_core_msgs/msg/RobotState.msg"
    "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/franka_ros_interface/franka_common/franka_core_msgs/msg/EndPointState.msg"
    "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/franka_ros_interface/franka_common/franka_core_msgs/msg/JointLimits.msg"
    "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/franka_ros_interface/franka_common/franka_core_msgs/msg/JointControllerStates.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_core_msgs/cmake" TYPE FILE FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/franka_ros_interface/franka_common/franka_core_msgs/catkin_generated/installspace/franka_core_msgs-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/devel/include/franka_core_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/devel/share/roseus/ros/franka_core_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/devel/share/common-lisp/ros/franka_core_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/devel/share/gennodejs/ros/franka_core_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/devel/lib/python3/dist-packages/franka_core_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/devel/lib/python3/dist-packages/franka_core_msgs")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/franka_ros_interface/franka_common/franka_core_msgs/catkin_generated/installspace/franka_core_msgs.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_core_msgs/cmake" TYPE FILE FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/franka_ros_interface/franka_common/franka_core_msgs/catkin_generated/installspace/franka_core_msgs-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_core_msgs/cmake" TYPE FILE FILES
    "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/franka_ros_interface/franka_common/franka_core_msgs/catkin_generated/installspace/franka_core_msgsConfig.cmake"
    "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/franka_ros_interface/franka_common/franka_core_msgs/catkin_generated/installspace/franka_core_msgsConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/franka_core_msgs" TYPE FILE FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/franka_ros_interface/franka_common/franka_core_msgs/package.xml")
endif()

