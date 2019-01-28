cmake_minimum_required(VERSION 3.5)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -g -O3 -std=c++1y")

include_directories(include)
set(gflags_dir "/home/julian/gflags-2.2.1/build")
set(glog_dir "/home/julian/glog-0.3.5/install")

# glog
add_library(glog INTERFACE)
target_include_directories(glog INTERFACE ${glog_dir}/include)
target_link_libraries(glog INTERFACE ${glog_dir}/lib/libglog.a)

# gflags
add_library(gflags INTERFACE)
target_include_directories(gflags INTERFACE ${gflags_dir}/include)
target_link_libraries(gflags INTERFACE ${gflags_dir}/lib/libgflags.a)

SET(LIBRARIES_PATH ${CMAKE_CURRENT_LIST_DIR}/lib)

find_library(PI_SENSORS pi_sensors
    PATHS ${LIBRARIES_PATH})

find_library(PI_UTILS pi_utils
    PATHS ${LIBRARIES_PATH})

add_executable(radar77G_ui_test src/ui/radar77G_ui_test.cpp)
target_link_libraries(radar77G_ui_test ${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_core.so ${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_highgui.so ${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_imgcodecs.so ${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_imgproc.so ${PI_SENSORS} ${PI_UTILS} usbcan glog pthread)

add_executable(sonar_ui_test src/ui/sonar_ui_test.cpp)
target_link_libraries(sonar_ui_test ${PI_SENSORS} ${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_core.so ${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_highgui.so ${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_imgcodecs.so ${CMAKE_CURRENT_LIST_DIR}/lib/libopencv_imgproc.so ${PI_UTILS} usbcan glog pthread)
