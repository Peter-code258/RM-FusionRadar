cmake_minimum_required(VERSION 3.16)
project(sdr_radio_decoder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${CMAKE_SOURCE_DIR}/common/include
)

# Core library
add_library(sdr_lib
    src/sdr_decoder.cpp
    src/udp_publisher.cpp
)

# Main executable
add_executable(sdr_decoder src/main.cpp)
target_link_libraries(sdr_decoder sdr_lib)

# Tests
enable_testing()
add_executable(test_sdr test/test_sdr.cpp)
target_link_libraries(test_sdr sdr_lib)
add_test(NAME test_sdr COMMAND test_sdr)

# Channel scan tool
add_executable(channel_scan tools/channel_scan.cpp)
target_link_libraries(channel_scan sdr_lib)

# Replay tool
add_executable(sdr_replay tools/sdr_replay.cpp)
target_link_libraries(sdr_replay sdr_lib)

file(COPY ${CMAKE_SOURCE_DIR}/config DESTINATION ${CMAKE_BINARY_DIR})

message(STATUS "SDR Radio Decoder build configured")
