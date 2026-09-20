cmake_minimum_required(VERSION 3.16)
project(anti_drone_tracking LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/common/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(anti_drone_lib src/anti_drone_tracker.cpp)
target_link_libraries(anti_drone_lib ${OpenCV_LIBS})

add_executable(anti_drone_tracker src/main.cpp)
target_link_libraries(anti_drone_tracker anti_drone_lib ${OpenCV_LIBS})

enable_testing()
add_executable(test_anti_drone test/test_anti_drone.cpp)
target_link_libraries(test_anti_drone anti_drone_lib ${OpenCV_LIBS})
add_test(NAME test_anti_drone COMMAND test_anti_drone)

file(COPY ${CMAKE_SOURCE_DIR}/config DESTINATION ${CMAKE_BINARY_DIR})

message(STATUS "Anti-Drone Tracking build configured")
