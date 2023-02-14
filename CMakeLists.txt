cmake_minimum_required(VERSION 3.20)
project(drone)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17 -g -Wall -Wextra -pedantic)

find_package(OpenCV REQUIRED)

add_executable(webcam webcam.cpp)
target_include_directories(webcam PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(webcam PUBLIC ${OpenCV_LIBS})

add_executable(webcamsafe webcamsafe.cpp)
target_include_directories(webcamsafe PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(webcamsafe PUBLIC ${OpenCV_LIBS})

# add_executable(test test.cpp)
# target_include_directories(test PUBLIC ${OpenCV_INCLUDE_DIRS})
# target_link_libraries(test PUBLIC ${OpenCV_LIBS})

# add_executable(calib camera_calibration.cpp)
# target_include_directories(calib PUBLIC ${OpenCV_INCLUDE_DIRS})
# target_link_libraries(calib PUBLIC ${OpenCV_LIBS})


