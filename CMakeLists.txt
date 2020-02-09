cmake_minimum_required(VERSION 3.6)
set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)

project(IMUBundleAdjustment)

# Options
set(LIBRARY_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libs CACHE PATH "Path to lib folder")
set(Eigen3_INCLUDE_DIR ${LIBRARY_DIR}/Eigen/ CACHE PATH "Path to Eigen source folder")

set(CMAKE_CXX_STANDARD 14)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (glog REQUIRED)
find_package (Ceres REQUIRED)
find_package( OpenCV REQUIRED )

include_directories(src ${CERES_INCLUDE_DIRS})
add_executable(IMUBundleAdjustment
        src/BundleAdjustment.cpp
        src/BundleAdjustment.h
        src/IMUIntegration.cpp
        src/IMUIntegration.h
        src/KeypointDetection.cpp
        src/KeypointDetection.h
        src/BundleAdjuster.cpp
        src/main.cpp
        src/CameraState.h
        src/BundleAdjuster.h)

add_executable(KeypointMatching
        src/KeypointMatching.cpp)

target_link_libraries(IMUBundleAdjustment Eigen3::Eigen glog::glog ${CERES_LIBRARIES})
target_link_libraries(KeypointMatching ${OpenCV_LIBS} )
