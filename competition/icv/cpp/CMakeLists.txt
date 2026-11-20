cmake_minimum_required(VERSION 3.16)
project(icv_cpp_reference LANGUAGES CXX)

add_library(icv_controller controller.cpp)
target_compile_features(icv_controller PUBLIC cxx_std_17)
target_include_directories(icv_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(icv_offline_demo offline_demo.cpp)
target_link_libraries(icv_offline_demo PRIVATE icv_controller)
