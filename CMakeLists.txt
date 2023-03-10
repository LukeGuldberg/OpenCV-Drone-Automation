cmake_minimum_required(VERSION 3.20)
project(drone)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17 -g -Wall -Wextra -pedantic)

find_package(OpenCV REQUIRED)

add_executable(webcam webcam.cpp hashpoints.cpp)
target_include_directories(webcam PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(webcam PUBLIC ${OpenCV_LIBS})

add_executable(test test.cpp hashpoints.cpp)
target_include_directories(test PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test PUBLIC ${OpenCV_LIBS})





