cmake_minimum_required(VERSION 3.15)
project(IMUBundleAdjustment)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE PATH "Path to lib folder")
set(Eigen3_INCLUDE_DIR ${LIBRARY_DIR}/Eigen/ CACHE PATH "Path to Eigen source folder")

set(CMAKE_CXX_STANDARD 14)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(src)
add_executable(IMUBundleAdjustment
        src/BundleAdjustment.cpp
        src/BundleAdjustment.h
        src/IMUIntegration.cpp
        src/IMUIntegration.h
        src/KeypointDetection.cpp
        src/KeypointDetection.h
        src/main.cpp src/CameraState.h)

target_link_libraries(IMUBundleAdjustment Eigen3::Eigen)
