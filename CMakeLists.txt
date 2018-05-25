cmake_minimum_required(VERSION 2.8.3)
project(orca_ros)

find_package(orca REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    eigen_conversions
    message_generation
    orca
)

add_message_files(
  FILES
  DesiredJoints.msg
)

add_service_files(
    FILES
    GetBool.srv
    SetBool.srv
    GetString.srv
    SetString.srv
    GetInt.srv
    SetInt.srv
    GetDouble.srv
    SetDouble.srv
    GetMatrix.srv
    SetMatrix.srv
    GetEnum.srv
    SetEnum.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
               INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
               LIBRARIES ${PROJECT_NAME}
               CATKIN_DEPENDS
               std_msgs
               geometry_msgs
               eigen_conversions
               message_runtime
            )


include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIRS}
                    )

file(GLOB_RECURSE source_files src/*)

add_library(${PROJECT_NAME} ${source_files})

target_link_libraries(  ${PROJECT_NAME}
                        ${catkin_LIBRARIES}
                        orca::orca
                    )
add_dependencies(${PROJECT_NAME} orca_ros_generate_messages_cpp)



add_executable(minimal_controller examples/minimal_controller.cc)
target_link_libraries(minimal_controller ${PROJECT_NAME})

# install(DIRECTORY include/${PROJECT_NAME}/
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
# 
# install(DIRECTORY launch
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
# 
# install(TARGETS
#         ${PROJECT_NAME}
#         ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#         RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#         )
