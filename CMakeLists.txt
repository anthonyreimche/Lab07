cmake_minimum_required(VERSION 3.30)
project(Lab07)

set(CMAKE_CXX_STANDARD 14)

add_executable(Lab07 main.cpp robot.cpp)

# Add Windows Socket library
target_link_libraries(Lab07 ws2_32)
