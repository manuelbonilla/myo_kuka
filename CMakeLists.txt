cmake_minimum_required(VERSION 2.8.3)
project(myo_kuka)
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
  calibration
)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

link_directories(${catkin_LIBRARY_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})


add_definitions(-std=c++11 --all-warnings -O3) 

find_library(KDL_LIBRARY REQUIRED NAMES orocos-kdl HINTS ${orocos_kdl_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# add a target to generate API documentation with Doxygen


# find_package(Doxygen)
# if(DOXYGEN_FOUND)
#   configure_file(${CMAKE_CURRENT_SOURCE_DIR}/doxygen.in @ONLY)
#   add_custom_target(doc
#   ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/doxygen.in
#   WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
#   COMMENT "Generating API documentation with Doxygen" VERBATIM)
# endif(DOXYGEN_FOUND)


# add_message_files(
#   FILES
# )

# add_service_files(
#   FILES
# )


# generate_messages(
#   DEPENDENCIES
#   std_msgs
#   geometry_msgs
#   sensor_msgs
#  )

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
  calibration
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})


add_library(${PROJECT_NAME} 
  src/teleoperation_controller.cpp 
  /home/manuelb/manuelb_ws/src/kuka-lwr/lwr_controllers/src/KinematicChainControllerBase.cpp
  )
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} kdl_parser ${orocos_kdl_LIBRARIES})

add_executable(hand_republisher src/hand_republisher.cpp)
add_dependencies(hand_republisher hand_republisher_gencpp ${PROJECT_NAME})
target_link_libraries(hand_republisher ${PROJECT_NAME} ${catkin_LIBRARIES})



install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

install(FILES myo_kuka_controllers.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})