cmake_minimum_required(VERSION 3.10)
project(orcahand_description)

if(NOT DEFINED ENV{ROS_VERSION} OR "$ENV{ROS_VERSION}" STREQUAL "1")
  message(STATUS "Detected ROS 1")

  find_package(catkin REQUIRED COMPONENTS
    urdf
    xacro
  )

  catkin_package()

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

  install(DIRECTORY urdf/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )

  install(DIRECTORY meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  )

  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )

else()
  message(STATUS "Detected ROS 2")

  find_package(ament_cmake REQUIRED)
  find_package(urdf REQUIRED)
  find_package(xacro REQUIRED)

  install(DIRECTORY urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
  )

  install(DIRECTORY meshes/
    DESTINATION share/${PROJECT_NAME}/meshes
  )

  install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
  )

  ament_package()
endif()
