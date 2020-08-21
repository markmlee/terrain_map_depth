cmake_minimum_required(VERSION 2.8.3)
project(gogo_gazelle)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-std=c++11)
else()
    add_compile_options(-std=c++11 -O3)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  nav_msgs
  actionlib
  actionlib_msgs
  geometry_msgs
)

add_action_files(
  DIRECTORY action
  FILES Motion.action

)

add_message_files(
  FILES
  update.msg

)

add_message_files(
  FILES
  update.msg
)

generate_messages(DEPENDENCIES actionlib_msgs std_msgs geometry_msgs)


include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

add_executable(gogo_gazelle src/wrapper.cpp src/lanros2podo.cpp src/lanpodo2ros.cpp)
target_link_libraries(gogo_gazelle ${catkin_LIBRARIES})

add_executable(test_footstep src/test_footstep.cpp src/lanros2podo.cpp src/lanpodo2ros.cpp)
target_link_libraries(test_footstep ${catkin_LIBRARIES})

add_executable(test_footstep_vision src/test_footstep_vision.cpp src/lanros2podo.cpp src/lanpodo2ros.cpp)
target_link_libraries(test_footstep_vision ${catkin_LIBRARIES})


add_executable(com_estimate src/com_estimate.cpp src/lanros2podo.cpp src/lanpodo2ros.cpp)
target_link_libraries(com_estimate ${catkin_LIBRARIES})

add_executable(detect_peak src/detect_peak.cpp src/lanros2podo.cpp src/lanpodo2ros.cpp)
target_link_libraries(detect_peak ${catkin_LIBRARIES})

add_executable(client src/client_depth.cpp)
target_link_libraries(client ${catkin_LIBRARIES})


add_executable(client_NP src/client_depth_NP.cpp)
target_link_libraries(client_NP ${catkin_LIBRARIES})


catkin_package(
    CATKIN_DEPENDS
    roscpp
    std_msgs
    actionlib
    actionlib_msgs
    geometry_msgs
)
