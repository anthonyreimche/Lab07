cmake_minimum_required(VERSION 3.25)

# Force MinGW compiler before project() call
set(CMAKE_C_COMPILER "C:/Program Files/JetBrains/CLion 2024.3.2/bin/mingw/bin/gcc.exe")
set(CMAKE_CXX_COMPILER "C:/Program Files/JetBrains/CLion 2024.3.2/bin/mingw/bin/g++.exe")

project(Lab07)

set(CMAKE_CXX_STANDARD 14)

add_executable(Lab07 main.cpp robot.cpp)

# Add Windows Socket library
if(WIN32)
    target_link_libraries(Lab07 ws2_32)
endif()
