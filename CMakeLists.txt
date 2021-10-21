cmake_minimum_required(VERSION 3.20)
project(historiymappub)

set(CMAKE_CXX_STANDARD 11)

# Opencv
find_package(OpenCV 3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# Eigen
find_package(Eigen3 REQUIRED)
include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(historiymappub main.cpp)
target_link_libraries(historiymappub ${OpenCV_LIBS})
