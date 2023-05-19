cmake_minimum_required(VERSION 3.0.2)
project(sensyn_dji_gimbal_action)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    genmsg
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf
    pcl_ros
    actionlib_msgs
    dji_osdk_ros
    )

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


#set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl/")
################################################
## Declare ROS messages, services and actions ##
################################################

# add_service_files(
#   FILES
# )

add_action_files(
  FILES
  ControlGimbal.action
  ShootCamera.action
  CameraParam.action
  GimbalCamera.action
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
  geometry_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
catkin_package(INCLUDE_DIRS include)
catkin_package(
INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
#  DEPENDS system_lib
 CATKIN_DEPENDS
# roscpp
# rospy
# std_msgs
# genmsg
# tf2
# tf2_ros
# tf2_geometry_msgs
dji_osdk_ros
)

###########
## Build ##
###########
include_directories(include
 ${catkin_INCLUDE_DIRS}
 ${PCL_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/fujita_tunnel.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/fujita_tunnel_node.cpp)

#add_executable(talker src/talker.cpp)
#add_executable(listener src/listener.cpp)
add_executable(dji_gimbal_action_server src/dji_gimbal_action_server.cpp)
add_executable(dji_camera_action_server src/dji_camera_action_server.cpp)
add_executable(dji_camera_param_action_server src/dji_camera_param_action_server.cpp)

add_executable(dji_gimbal_camera_action_server src/dji_gimbal_camera_action_server.cpp)

add_library(${PROJECT_NAME} src/dji_gimbal_camera_action_client.cpp )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})




## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
#target_link_libraries(talker ${catkin_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(listener ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(dji_gimbal_action_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} dji_osdk_ros)
target_link_libraries(dji_camera_action_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} dji_osdk_ros)
target_link_libraries(dji_camera_param_action_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} dji_osdk_ros)
target_link_libraries(dji_gimbal_camera_action_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} dji_osdk_ros)
# target_link_libraries(dji_gimbal_camera_action_client ${catkin_LIBRARIES} ${PCL_LIBRARIES} dji_osdk_ros)


link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


