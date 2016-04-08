cmake_minimum_required(VERSION 2.8.3)
project(bb8_driver)

set(ROSDEP_DEPS bluepy)

find_package(catkin REQUIRED)
catkin_package(DEPENDS ${ROSDEP_DEPS})

catkin_python_setup()

install(PROGRAMS scripts/test_bb8_driver.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
