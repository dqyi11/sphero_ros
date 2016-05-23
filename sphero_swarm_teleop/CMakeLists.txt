cmake_minimum_required(VERSION 2.8.3)
project(sphero_swarm_teleop)

find_package(catkin REQUIRED COMPONENTS
             rospy
             std_msgs
            )


catkin_package(
)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS src/sphero_swarm_teleop_gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/sphero_swarm_random_move.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
