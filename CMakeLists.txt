cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O3 -std=c++1y")

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIR})

include_directories(include)
SET(LIBRARIES_PATH ${CMAKE_CURRENT_LIST_DIR}/lib)

find_library(LIBOPENCV_CORE opencv_core
    PATHS ${LIBRARIES_PATH})

find_library(LIBOPENCV_HIGHGUI opencv_highgui
    PATHS ${LIBRARIES_PATH})

find_library(LIBOPENCV_IMGCODECS opencv_imgcodecs
    PATHS ${LIBRARIES_PATH})

find_library(LIBOPENCV_IMGPROC opencv_imgproc
    PATHS ${LIBRARIES_PATH})

find_library(PI_SENSORS pi_sensors
    PATHS ${LIBRARIES_PATH})

find_library(PI_UTILS pi_utils
    PATHS ${LIBRARIES_PATH})

add_executable(radar77G_ui_test src/ui/radar77G_ui_test.cpp)
target_link_libraries(radar77G_ui_test ${LIBOPENCV_CORE} ${LIBOPENCV_HIGHGUI} ${LIBOPENCV_IMGCODECS} ${LIBOPENCV_IMGPROC} ${PI_SENSORS} ${PI_UTILS} usbcan glog pthread)

add_executable(sonar_ui_test src/ui/sonar_ui_test.cpp)
target_link_libraries(sonar_ui_test ${LIBOPENCV_CORE} ${LIBOPENCV_HIGHGUI} ${LIBOPENCV_IMGCODECS} ${LIBOPENCV_IMGPROC} ${PI_SENSORS} ${PI_UTILS} usbcan glog pthread)
