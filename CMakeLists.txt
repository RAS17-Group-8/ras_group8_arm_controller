cmake_minimum_required(VERSION 2.8.3)
project(ras_group8_arm_controller)

find_package(catkin REQUIRED
  COMPONENTS
  roscpp
  uarm
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
  MoveArm.srv
)

generate_messages(
 DEPENDENCIES
  geometry_msgs
)


catkin_package(
  INCLUDE_DIRS
    include

  CATKIN_DEPENDS
    roscpp
    uarm
    std_msgs
    sensor_msgs
    geometry_msgs
    message_runtime

# DEPENDS
)





add_executable(${PROJECT_NAME}
  src/arm_controller_node.cpp
  src/ArmController.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

catkin_add_gtest(arm_controller_tests test/test_arm_controller.cpp src/ArmController.cpp)

# Make main executable
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)
