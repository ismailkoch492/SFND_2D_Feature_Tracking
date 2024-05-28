cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(camera_fusion)

set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV 4.1 REQUIRED)
# https://www.appsloveworld.com/cplus/100/1017/cmake-cannot-find-opencvconfig-cmake-even-after-specifying-opencv-dir
if (NOT "${OpenCV_FOUND}")
    unset(OpenCV_DIR)
    find_package(OpenCV 4.1 REQUIRED)
    # OpenCV_DIR = /lib/x86_64-linux-gnu/cmake/opencv4 -> OpenCVConfig.cmake
    message(STATUS "Checkout the solution: https://github.com/opencv/opencv/issues/21778")
else ()
    set(OPENCV_ENABLE_NONFREE 1)
endif ()

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_DEFINITIONS})

# Executable for create matrix exercise
add_executable (2D_feature_tracking src/matching2D_Student.cpp src/MidTermProject_Camera_Student.cpp)
target_link_libraries (2D_feature_tracking ${OpenCV_LIBRARIES})

add_executable (test src/test/test_matching2D_Student.cpp src/test/test_MTProject.cpp)
target_link_libraries (test ${OpenCV_LIBRARIES})