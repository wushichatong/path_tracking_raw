cmake_minimum_required(VERSION 3.5.2)
project(tracking_raw)
add_compile_options(-std=c++11)


# add_executable(interpolator src/controller.cpp src/path_interpolator.cpp src/cubic_spline_interpolator.cpp src/path_interpolator_test.cpp)
add_executable(interpolator src/controller.cpp src/cubic_spline_interpolator.cpp src/path_interpolator_test.cpp)
target_link_libraries(interpolator)