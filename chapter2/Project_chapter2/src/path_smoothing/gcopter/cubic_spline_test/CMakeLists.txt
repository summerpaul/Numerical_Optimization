add_executable(cubic_spline_test_node cubic_spline_test_node.cpp cubic_spline_test.cpp)

target_link_libraries(cubic_spline_test_node
  ${catkin_LIBRARIES}
)