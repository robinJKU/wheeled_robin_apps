cmake_minimum_required(VERSION 2.8.3)
project(wheeled_robin_navigation)

find_package(catkin REQUIRED COMPONENTS
  turtlebot_navigation
)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY param
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

