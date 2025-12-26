cmake_minimum_required(VERSION 3.15)
project(my-project)

find_package(Eigen3 CONFIG REQUIRED)

add_executable(ekf-attitude-estimator    
    src/main.cpp
    src/KalmanFilter.cpp
    src/Quaternion.cpp)
    
target_include_directories(ekf-attitude-estimator
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/inc)

target_link_libraries(ekf-attitude-estimator 
    PRIVATE 
        Eigen3::Eigen)
