cmake_minimum_required(VERSION 2.8.3)

project(wheeled_robin_formation_drive)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY data
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY src
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_executable(marker_validation src/marker_val.cpp)
target_link_libraries(marker_validation ${catkin_LIBRARIES})

add_executable(vel_control src/vel_control.cpp)
target_link_libraries(vel_control ${catkin_LIBRARIES})
