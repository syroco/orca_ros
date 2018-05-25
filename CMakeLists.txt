cmake_minimum_required(VERSION 2.8.3)
project(orca_ros)

find_package(orca REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    geometry_msgs
    eigen_conversions
    message_generation
)

add_message_files(
  FILES
  DesiredJoints.msg
)

# add_service_files(
#   FILES
#   _________.srv
# )

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  eigen_conversions
)

catkin_package(
               INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIRS}
               DEPENDS orca::orca
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

add_library(    ${PROJECT_NAME}
                src/*/*.cc
            )

target_link_libraries(  ${PROJECT_NAME}
                        ${catkin_LIBRARIES}
                        orca::orca
                    )



add_dependencies(${PROJECT_NAME} orca_msgs_generate_messages_cpp)



add_executable(orca_controller src/orca_controller.cc)
target_link_libraries(orca_controller ${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS
        ${PROJECT_NAME}
        orca_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        )
