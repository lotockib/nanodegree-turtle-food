cmake_minimum_required(VERSION 3.0.2)
project(turtle_food)

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)

find_package(Qt5Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_message_files(DIRECTORY msg FILES
                  Color.msg Pose.msg)
# add_service_files(DIRECTORY srv FILES
#                   Kill.srv
#                   SetPen.srv
#                   Spawn.srv
#                   TeleportAbsolute.srv
#                   TeleportRelative.srv)
add_service_files(
                  FILES
                  Kill.srv
                  KillFood.srv
                  SetPen.srv
                  Spawn.srv
                  SpawnFood.srv
                  TeleportAbsolute.srv
                  TeleportRelative.srv)

generate_messages(DEPENDENCIES geometry_msgs std_msgs std_srvs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

set(turtlesim_node_SRCS
  src/turtlesim.cpp
  src/turtle.cpp
  src/turtle_frame.cpp
)
set(turtlesim_node_HDRS
  include/turtle_food/turtle_frame.h
)

qt5_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})

# set(food_node_SRCS
#   src/food.cpp
# )

set(food_node_HDRS
  include/turtle_food/food.h
)

qt5_wrap_cpp(food_node_MOCS ${food_node_HDRS})

add_executable(turtlesim_node ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
target_link_libraries(turtlesim_node Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(turtlesim_node turtle_food_generate_messages_cpp)

add_executable(turtle_teleop_key tutorials/teleop_turtle_key.cpp)
target_link_libraries(turtle_teleop_key ${catkin_LIBRARIES})
add_dependencies(turtle_teleop_key turtle_food_generate_messages_cpp)
add_executable(draw_square tutorials/draw_square.cpp)
target_link_libraries(draw_square ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(draw_square turtle_food_generate_messages_cpp)
add_executable(mimic tutorials/mimic.cpp)
target_link_libraries(mimic ${catkin_LIBRARIES})
add_dependencies(mimic turtle_food_generate_messages_cpp)

# add_executable(food_node ${food_node_SRCS} ${food_node_HDRS})
# target_link_libraries(food_node ${catkin_LIBRARIES})
# add_dependencies(food_node turtle_food_generate_messages_cpp)

add_executable(food_node src/food.cpp ${food_node_MOCS})
target_link_libraries(food_node Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(food_node turtle_food_generate_messages_cpp)

install(TARGETS turtlesim_node turtle_teleop_key draw_square mimic food_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY images
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")
