cmake_minimum_required(VERSION 3.16)
enable_language(CXX)
set(CMAKE_CXX_STANDARD 20) # C++20...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS OFF) #...without compiler extensions like gnu++11

project(DDT-MOTOR)

file(GLOB src_files src/*.cpp)

add_library(ddtmotor ${src_files})
target_include_directories(ddtmotor PUBLIC include)

target_compile_options(ddtmotor PRIVATE -O3 -Wall -Wextra -Wno-long-long -pedantic)

add_subdirectory(examples)