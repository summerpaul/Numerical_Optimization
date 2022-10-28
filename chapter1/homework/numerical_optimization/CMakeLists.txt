cmake_minimum_required(VERSION 3.0.0)
project(numerical_optimization)
find_package(Eigen3 REQUIRED)
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    
)
add_subdirectory(gradient_descent)
add_subdirectory(test)