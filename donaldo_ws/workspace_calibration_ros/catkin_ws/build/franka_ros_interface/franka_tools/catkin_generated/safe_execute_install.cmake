execute_process(COMMAND "/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/franka_ros_interface/franka_tools/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/lab/donaldo_ws/workspace_calibration_ros/catkin_ws/build/franka_ros_interface/franka_tools/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
