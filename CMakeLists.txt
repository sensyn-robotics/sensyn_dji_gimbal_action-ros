cmake_minimum_required(VERSION 3.0.2)
project(sensyn_dji_gimbal_action)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    genmsg
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf
    actionlib_msgs
    dji_osdk_ros
    )

find_package(Eigen3 REQUIRED)


add_action_files(
  FILES
  GimbalCamera.action
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  actionlib_msgs
  geometry_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
INCLUDE_DIRS include
 LIBRARIES ${PROJECT_NAME}
#  DEPENDS system_lib
 CATKIN_DEPENDS roscpp std_msgs dji_osdk_ros
# roscpp
# rospy
# std_msgs
# genmsg
# tf2
# tf2_ros
# tf2_geometry_msgs
# dji_osdk_ros
)

###########
## Build ##
###########
include_directories(include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIR}
)


add_executable(dji_gimbal_camera_action_server src/dji_gimbal_camera_action_server.cpp)
add_dependencies(dji_gimbal_camera_action_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dji_gimbal_camera_action_server ${catkin_LIBRARIES})


add_library(${PROJECT_NAME} src/dji_gimbal_camera_action_client.cpp )

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )


install(TARGETS dji_gimbal_camera_action_server ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
FILES_MATCHING PATTERN "*.h"
PATTERN ".svn" EXCLUDE
)
