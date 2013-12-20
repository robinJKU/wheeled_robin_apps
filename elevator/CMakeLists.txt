cmake_minimum_required(VERSION 2.8.3)
project(elevator)

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

add_executable(state_machine_elevator src/state_machine_elevator.cpp)
## add_dependencies(detect_open_door ${open_door_detector}_generate_messages_cpp)

target_link_libraries(state_machine_elevator
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############

