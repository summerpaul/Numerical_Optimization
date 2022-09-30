cmake_minimum_required(VERSION 3.1)
project(Nlopt-Learning)

set (CMAKE_CXX_STANDARD 11)

find_package(NLopt REQUIRED)

include_directories(
    ${NLopt_INCLUDE_DIRS}
)
 
#nlopt_test
add_executable(nlopt_test src/nlopt_test.cpp)
target_link_libraries(nlopt_test ${NLOPT_LIBRARIES})
