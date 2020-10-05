execute_process(COMMAND "/home/sibohao/Desktop/master_arbeit/build/preparing/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/sibohao/Desktop/master_arbeit/build/preparing/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
