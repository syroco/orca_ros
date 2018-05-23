cmake_minimum_required(VERSION 2.8.3)
project(orca_ros)

find_package(catkin REQUIRED)

find_package(orca REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 DEPENDS orca::orca
)

include_directories(

  ${orca_INCLUDE_DIRS}
)
