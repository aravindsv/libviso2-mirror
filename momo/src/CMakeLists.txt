cmake_minimum_required(VERSION 2.6)

include_directories(../include/momo)
add_library(momo motion_optimizer.cpp outlier_detector.cpp)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
target_link_libraries(momo Eigen3::Eigen)
target_link_libraries(momo ${CERES_LIBRARIES})