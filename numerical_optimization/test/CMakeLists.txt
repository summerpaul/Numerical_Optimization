
include_directories(
/usr/include/python3.8)
add_executable(chapter1_test chapter1_test)
target_link_libraries(chapter1_test gradient_descent python3.8)