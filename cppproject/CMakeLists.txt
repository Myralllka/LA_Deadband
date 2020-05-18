cmake_minimum_required(VERSION 3.16)
project(cppproject)
set(CMAKE_CXX_STANDARD 20)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(cppproject main.cpp)

target_link_libraries (cppproject Eigen3::Eigen)
