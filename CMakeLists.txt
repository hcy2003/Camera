cmake_minimum_required(VERSION 3.10)
project(Camera)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=c++14 -pthread")

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

#引入头文件
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(/opt/MVS/include/)
link_directories(/opt/MVS/lib/64/)

add_executable(Camera changeCamera.cpp
            ${PROJECT_SOURCE_DIR}/serial_port.cpp
            ${PROJECT_SOURCE_DIR}/general.cpp
            ${PROJECT_SOURCE_DIR}/HKCamera.cpp
)

#把库包含进来
target_link_libraries(Camera ${OpenCV_LIBRARIES} 
                            libMvCameraControl.so
                                )


