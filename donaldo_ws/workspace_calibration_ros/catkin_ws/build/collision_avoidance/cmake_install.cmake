# Install script for directory: /home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/collision_avoidance/srv" TYPE FILE FILES
    "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/CalcStopDuration.srv"
    "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/srv/FlagStop.srv"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/collision_avoidance/cmake" TYPE FILE FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/collision_avoidance/catkin_generated/installspace/collision_avoidance-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/devel/include/collision_avoidance")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/devel/share/roseus/ros/collision_avoidance")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/devel/share/common-lisp/ros/collision_avoidance")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/devel/share/gennodejs/ros/collision_avoidance")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/devel/lib/python3/dist-packages/collision_avoidance")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/devel/lib/python3/dist-packages/collision_avoidance")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/collision_avoidance/catkin_generated/installspace/collision_avoidance.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/collision_avoidance/cmake" TYPE FILE FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/collision_avoidance/catkin_generated/installspace/collision_avoidance-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/collision_avoidance/cmake" TYPE FILE FILES
    "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/collision_avoidance/catkin_generated/installspace/collision_avoidanceConfig.cmake"
    "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/collision_avoidance/catkin_generated/installspace/collision_avoidanceConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/collision_avoidance" TYPE FILE FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/src/collision_avoidance/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/collision_avoidance" TYPE PROGRAM FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/collision_avoidance/catkin_generated/installspace/base_rotation_matrix_subscriber.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/collision_avoidance" TYPE PROGRAM FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/collision_avoidance/catkin_generated/installspace/calc_stop_duration_server.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/collision_avoidance" TYPE PROGRAM FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/collision_avoidance/catkin_generated/installspace/demo_stop_event_traj.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/collision_avoidance" TYPE PROGRAM FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/collision_avoidance/catkin_generated/installspace/flag_stop_server.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/collision_avoidance" TYPE PROGRAM FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/collision_avoidance/catkin_generated/installspace/marker_pos_calculator.py")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/collision_avoidance" TYPE PROGRAM FILES "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/collision_avoidance/catkin_generated/installspace/base_rotation_matrix_modificato.py")
endif()

