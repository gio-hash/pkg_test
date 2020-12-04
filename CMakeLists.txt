cmake_minimum_required(VERSION 2.8.3)
project(pkg_test)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    roscpp
    rospy
    geometry_msgs
    std_msgs
    trajectory_msgs
    lwr_hw
    )

find_package(Boost REQUIRED COMPONENTS
    system
    chrono
    timer
    )
find_package(Eigen3 REQUIRED)


catkin_package(LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    CATKIN_DEPENDS message_runtime
    roscpp
    rospy
    geometry_msgs
    std_msgs
    

    )

add_library(${PROJECT_NAME}
    src/${PROJECT_NAME}_node.cpp
    
    
    )

add_dependencies(${PROJECT_NAME}
    ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_generate_messages_cpp
    ${PROJECT_NAME}_gencpp
    )

include_directories(include/
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    )

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )
target_link_libraries(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME} PUBLIC
    include
    )

add_executable(my_controller_node
    src/my_controller_node.cpp
    src/MyController.cpp
    src/KinematicChainControllerBase.cpp )
target_link_libraries(my_controller_node ${catkin_LIBRARIES} )
add_dependencies(my_controller_node ${PROJECT_NAME}_node_generate_messages_cpp)