cmake_minimum_required(VERSION 3.5.2)
project(robust_optimization)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14")
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Boost REQUIRED COMPONENTS system filesystem regex timer date_time program_options)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_subdirectory(robust_g2o)
add_subdirectory(robust_gtsam)
add_subdirectory(evaluator)
