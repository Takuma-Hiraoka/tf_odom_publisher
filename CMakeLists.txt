cmake_minimum_required(VERSION 3.0.2)
project(tf_odom_publisher)

set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  tf2_ros
  )
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3)

catkin_package(
)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
 )

add_executable(tf_odom_publisher src/tf_odom_publisher.cpp)

target_link_libraries(tf_odom_publisher
  ${catkin_LIBRARIES}
 )

install(TARGETS tf_odom_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
