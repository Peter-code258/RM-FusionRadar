cmake_minimum_required(VERSION 3.16)
project(multi_sensor_fusion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/common/include
    ${EIGEN3_INCLUDE_DIR}
)

add_library(fusion_lib
    src/fusion_center.cpp
    src/udp_io.cpp
)
target_link_libraries(fusion_lib pthread)

add_executable(fusion_center src/main.cpp)
target_link_libraries(fusion_center fusion_lib)

enable_testing()
add_executable(test_fusion test/test_fusion.cpp)
target_link_libraries(test_fusion fusion_lib)
add_test(NAME test_fusion COMMAND test_fusion)

file(COPY ${CMAKE_SOURCE_DIR}/config DESTINATION ${CMAKE_BINARY_DIR})

message(STATUS "Multi-Sensor Fusion build configured")
