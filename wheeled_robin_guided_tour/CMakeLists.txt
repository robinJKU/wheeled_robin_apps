cmake_minimum_required(VERSION 2.8.3)
project(wheeled_robin_guided_tour)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  move_base_msgs
  roscpp
  std_msgs
  tf
)

catkin_package(

)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(state_machine src/state_machine.cpp)
add_dependencies(state_machine ${video_player}_generate_messages_cpp)
target_link_libraries(state_machine
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############

