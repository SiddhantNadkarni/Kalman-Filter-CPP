cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 11)

project (KalmanFilter)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(SOURCES src/main.cpp src/KalmanFilter.cpp
)


add_executable(KalmanFilter ${SOURCES})

target_link_libraries(KalmanFilter Eigen3::Eigen)
target_include_directories(KalmanFilter 
    PRIVATE 
        ${PROJECT_SOURCE_DIR}/include 
)