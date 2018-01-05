cmake_minimum_required(VERSION 2.8)
project( ASTAR )

# set compiler
set( CMAKE_CXX_COMPILER "g++")
set( CMAKE_CXX_FLAGS "-std=c++11 -O3")

# Set the directory of binaries
set( EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 

include_directories(${PROJECT_SOURCE_DIR}/include)

file(GLOB SOURCES "src/*.cpp")
add_executable( astar ${SOURCES} )
