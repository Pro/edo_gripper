cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(edo_gripper_plugin)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        gazebo_ros
        )

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)


include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(edo_gripper SHARED edo_gripper_plugin.cpp)

target_link_libraries(edo_gripper ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

