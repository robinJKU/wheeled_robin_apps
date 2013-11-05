cmake_minimum_required(VERSION 2.8.3)
project(wheeled_robin_teleop)

find_package(catkin REQUIRED COMPONENTS
  joy
)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


