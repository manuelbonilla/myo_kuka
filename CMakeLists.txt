cmake_minimum_required(VERSION 2.8.3)
project(teleoperation_controllers)
find_package(orocos_kdl REQUIRED)
find_package(catkin REQUIRED 
  sensor_msgs
  controller_interface
  control_msgs
  forward_command_controller
  control_toolbox
  realtime_tools
  urdf
  roscpp
  kdl_parser
  message_generation
  cmake_modules
  roscpp 
  tf_conversions 
  message_generation 
  std_msgs
  geometry_msgs 
  sensor_msgs
  lwr_controllers
)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})


add_definitions(-std=c++11 --all-warnings -O3) 

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

catkin_package(
  DEPENDS orocos_kdl
  CATKIN_DEPENDS
  controller_interface
  control_msgs
  control_toolbox
  realtime_tools
  urdf
  forward_command_controller
  kdl_parser
  message_runtime
  tf_conversions
  lwr_hw
  lwr_controllers
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})


add_library(${PROJECT_NAME} src/slave_test_controller.cpp src/test_control.cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES teleoperation_controllers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})