cmake_minimum_required(VERSION 2.8.3)
project(orca_ros)

find_package(orca REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    sensor_msgs
    eigen_conversions
    message_generation
    orca
)
find_package(gazebo REQUIRED)

if(${GAZEBO_VERSION} VERSION_GREATER "7.0")
    message(STATUS "Gazebo ${GAZEBO_VERSION} found, building examples")

    include_directories(${GAZEBO_INCLUDE_DIRS})
    link_directories(${GAZEBO_LIBRARY_DIRS})
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

    add_definitions(-DGAZEBO_VERSION_MAJOR=${gazebo_VERSION_MAJOR})

    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "GNU")
        add_definitions(-fext-numeric-literals)
    endif()
else()
    message(FATAL_ERROR "Gazebo ${GAZEBO_VERSION} found, we need at least gazebo 7")
endif()

add_message_files(
  FILES
  RobotState.msg
  JointTorqueCommand.msg
  TaskDescription.msg
  ConstraintDescription.msg
  CartesianTaskState.msg
)

add_service_files(
    FILES
    GetBool.srv
    SetBool.srv
    GetString.srv
    SetString.srv
    GetInt.srv
    GetStringList.srv
    SetInt.srv
    GetDouble.srv
    SetDouble.srv
    GetMatrix.srv
    SetMatrix.srv
    GetEnum.srv
    SetEnum.srv
    GetSize.srv
    SetSize.srv
    AddTask.srv
    AddConstraint.srv
    UpdateController.srv
    GetDesired.srv
    SetDesired.srv
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

set(sub-dirs
    optim
    constraint
    common
    #math
    robot
    task
    utils
)

foreach(sub : ${sub-dirs})
    file(GLOB sub-srcs src/${sub}/*.cc)
    list(APPEND source_files ${sub-srcs})
endforeach()

add_library(${PROJECT_NAME} ${source_files})

target_link_libraries(  ${PROJECT_NAME}
                        ${catkin_LIBRARIES}
                        orca::orca
                    )
add_dependencies(${PROJECT_NAME} orca_ros_generate_messages_cpp)

# Example nodes:
add_executable(minimal_controller examples/minimal_controller.cc)
target_link_libraries(minimal_controller ${PROJECT_NAME})

add_executable(minimal_client examples/minimal_client.cc)
target_link_libraries(minimal_client ${PROJECT_NAME})

add_library(${PROJECT_NAME}_gazebo src/gazebo/RosGazeboModel.cc)
target_link_libraries(${PROJECT_NAME}_gazebo ${PROJECT_NAME} ${GAZEBO_LIBRARIES} pthread tinyxml)
add_executable(gazebo_node examples/gazebo_node.cc)
target_link_libraries(gazebo_node ${PROJECT_NAME}_gazebo)

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
