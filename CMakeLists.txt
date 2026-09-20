cmake_minimum_required(VERSION 3.16)
project(lidar_perception LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters segmentation visualization search kdtree)
find_package(Eigen3 REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/common/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Core library
add_library(lidar_perception_lib
    src/lidar_perception.cpp
)
target_link_libraries(lidar_perception_lib ${PCL_LIBRARIES})

# Main demo executable
add_executable(lidar_perception src/main.cpp)
target_link_libraries(lidar_perception lidar_perception_lib ${PCL_LIBRARIES})

# Tests
enable_testing()
add_executable(test_lidar test/test_lidar.cpp)
target_link_libraries(test_lidar lidar_perception_lib ${PCL_LIBRARIES})
add_test(NAME test_lidar COMMAND test_lidar)

# Copy config to build dir
file(COPY ${CMAKE_SOURCE_DIR}/config DESTINATION ${CMAKE_BINARY_DIR})

message(STATUS "LiDAR Perception build configured")
