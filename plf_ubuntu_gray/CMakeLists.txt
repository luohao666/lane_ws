cmake_minimum_required(VERSION 2.8)
project( PoweLaneFinding )

#set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -pthread -lstdc++ -lm")

# 添加Eigen头文件
#include_directories(include)
#include_directories( "/usr/local/include/eigen3" )

find_package( OpenCV REQUIRED )

add_executable( lane_gray main.cpp)
target_link_libraries( lane_gray ${OpenCV_LIBS} )

add_executable( lane_gray_imgs main2.cpp)
target_link_libraries( lane_gray_imgs ${OpenCV_LIBS} )


