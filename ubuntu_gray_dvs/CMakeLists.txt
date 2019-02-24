cmake_minimum_required(VERSION 2.8)
project( PoweLaneFinding )

#set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -pthread -lstdc++ -lm")

find_package( OpenCV REQUIRED )

include_directories( 
    include
    ${OpenCV_INCLUDE_DIRS} )

file(GLOB SOURCES "src/*.cpp")


add_executable( lane ${SOURCES} )

target_link_libraries( lane ${OpenCV_LIBS} )
