cmake_minimum_required(VERSION 2.8.3)
project(sphero_swarm_node)

find_package(catkin REQUIRED message_generation dynamic_reconfigure std_msgs geometry_msgs)

add_message_files(FILES
                  SpheroSwarmCollision.msg
                  SpheroOdometry.msg
                  SpheroImu.msg
                  SpheroTwist.msg
                  SpheroTurn.msg
                  SpheroColor.msg
                  SpheroBackLed.msg
                  SpheroDisableStabilization.msg
                  SpheroHeading.msg
                  SpheroAngularVelocity.msg
                 )

add_service_files(
  FILES
  SpheroInfo.srv
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)
generate_dynamic_reconfigure_options(cfg/Reconfig.cfg)

catkin_package(DEPENDS std_msgs geometry_msgs dynamic_reconfigure message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

install(PROGRAMS src/sphero_swarm.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS src/sphero_swarm_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
