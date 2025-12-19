execute_process(COMMAND "/home/vboxuser/final_project/catkin_ws/build/tracker_pkg/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/vboxuser/final_project/catkin_ws/build/tracker_pkg/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
