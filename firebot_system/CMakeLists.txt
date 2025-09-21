cmake_minimum_required(VERSION 3.0.2)
add_compile_options(-std=c++17)
project(firebot_system)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  nav_msgs
  tf
)

add_message_files(
  FILES
    Sonars.msg
)

generate_messages(
  DEPENDENCIES
  gazebo_msgs
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(scout_node src/scout_node.cpp)
target_link_libraries(scout_node ${catkin_LIBRARIES})
add_dependencies(scout_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(map_node src/map_node.cpp)
target_link_libraries(map_node ${catkin_LIBRARIES})
add_dependencies(map_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(firebot_executor_node src/firebot_executor_node.cpp)
target_link_libraries(firebot_executor_node ${catkin_LIBRARIES})
add_dependencies(firebot_executor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(firebot_sonars src/sonars.cc)
target_link_libraries(firebot_sonars ${catkin_LIBRARIES})
add_dependencies(firebot_sonars assignment1_setup_generate_messages_cpp)
set_target_properties(firebot_sonars PROPERTIES OUTPUT_NAME sonars)
