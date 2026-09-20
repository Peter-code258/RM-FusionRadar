cmake_minimum_required(VERSION 3.16)
project(mono_vision_radar LANGUAGES CXX)

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

add_library(mono_lib src/mono_radar.cpp)
target_link_libraries(mono_lib ${OpenCV_LIBS})

add_executable(mono_radar src/main.cpp)
target_link_libraries(mono_radar mono_lib ${OpenCV_LIBS})

enable_testing()
add_executable(test_mono test/test_mono.cpp)
target_link_libraries(test_mono mono_lib ${OpenCV_LIBS})
add_test(NAME test_mono COMMAND test_mono)

add_executable(calibration_tool tools/calibration_tool.cpp)
target_link_libraries(calibration_tool mono_lib ${OpenCV_LIBS})

file(COPY ${CMAKE_SOURCE_DIR}/config DESTINATION ${CMAKE_BINARY_DIR})

message(STATUS "Monocular Vision Radar build configured")
