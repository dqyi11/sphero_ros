cmake_minimum_required(VERSION 2.8.3)
project(sphero_swarm_bringup)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


