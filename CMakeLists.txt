cmake_minimum_required(VERSION 3.1.0)
project(rpi-control)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Threads REQUIRED)

# Bring the headers
# include_directories(arch/linux)
include_directories(include)

# Can manually add the sources using the set command as follows:
set(SOURCES src/user_ai.cpp src/control.cpp)
# src/state_machine.cpp
# set (CMAKE_CXX_STANDARD 11) # in CMAKE 3.1.0+

# However, the file(GLOB...) allows for wildcard additions:
# file(GLOB SOURCES "./*.c")

add_executable(rpi-control ${SOURCES})

# required for forcing -lpthread
target_link_libraries(rpi-control ${CMAKE_THREAD_LIBS_INIT} -lrt)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug)
endif()

