cmake_minimum_required(VERSION 3.16)
project(lidar_perception LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Find dependencies (core PCL components without visualization)
# Note: visualization component is excluded due to VTK compatibility issues on this platform
find_package(PCL 1.12 REQUIRED COMPONENTS common io filters segmentation search kdtree)
find_package(Eigen3 REQUIRED)

set(HAS_PCL_VISUALIZATION OFF)
message(STATUS "PCL visualization disabled - demo runs headless")

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/common/include
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Core library (no visualization dependency)
add_library(lidar_perception_lib
    src/lidar_perception.cpp
)
target_link_libraries(lidar_perception_lib ${PCL_LIBRARIES})

# Main demo executable
add_executable(lidar_perception src/main.cpp)
target_link_libraries(lidar_perception lidar_perception_lib ${PCL_LIBRARIES})
if(HAS_PCL_VISUALIZATION)
    target_compile_definitions(lidar_perception PRIVATE HAS_PCL_VISUALIZATION=1)
endif()

# Tests
enable_testing()
add_executable(test_lidar test/test_lidar.cpp)
target_link_libraries(test_lidar lidar_perception_lib ${PCL_LIBRARIES})
add_test(NAME test_lidar COMMAND test_lidar)

# Copy config to build dir
file(COPY ${CMAKE_SOURCE_DIR}/config DESTINATION ${CMAKE_BINARY_DIR})

message(STATUS "LiDAR Perception build configured (visualization: ${HAS_PCL_VISUALIZATION})")
