cmake_minimum_required(VERSION 3.0)

project(RobotControl)

# Set the path to Eigen (modify this path based on where you extracted Eigen)
set(EIGEN_PATH "C:/Program Files/eigen-3.4.0")

include_directories(${EIGEN_PATH})

add_executable(Eigen_test Eigen_test.cpp)