cmake_minimum_required(VERSION 2.8.3)
project(hubo_planner)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_options(-std=c++11)
else()
    add_compile_options(-std=c++11 -O3)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    octomap_msgs
    visualization_msgs
    gogo_gazelle
    message_generation
    tf
)

## System dependencies are found with CMake's conventions
find_package(octomap REQUIRED)
FIND_PACKAGE(superray_quadmap REQUIRED)
# find_package(PCL REQUIRED)
# find_package(Eigen3 REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    Footstep.msg
    FootstepsStamped.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    geometry_msgs
    std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS include
#  LIBRARIES hubo_planner
#    CATKIN_DEPENDS roscpp message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${QUADMAP_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${gogo_gazelle_INCLUDE_DIRS}
)

SET(COLLISION_DETECTOR_SRC
    src/collision_detector/collision_detector.cpp
    include/collision_detector/collision_detector.h
    include/collision_detector/primitive_aabb.h
    include/collision_detector/primitive_obb.h
    include/collision_detector/primitive_circle.h
    include/collision_detector/primitive_box.h
)

add_executable(footstep_planning
    src/footstep_planning.cpp
    src/footstep_planner/footstep_planner.cpp

    include/footstep_planner/footstep_planner.h
    include/hubo_planner/utils.h
    include/footstep_planner/footstep_planner_node.h

    ${COLLISION_DETECTOR_SRC}
)
target_link_libraries(footstep_planning
    ${catkin_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${QUADMAP_LIBRARIES}
)
add_dependencies(footstep_planning
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)

add_executable(footstep_subscribe_example
    src/footstep_subscribe_example.cpp
)
target_link_libraries(footstep_subscribe_example
    ${catkin_LIBRARIES}
)
