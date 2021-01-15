cmake_minimum_required(VERSION 3.1.0)
project(natnet-fwd)

# set (CMAKE_CXX_STANDARD 11) # in CMAKE 3.1.0+

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Threads REQUIRED)

# Bring the headers
include_directories(include/arch/linux)
include_directories(include)

# compile the libraries for linux architecture like udp socket
add_library(UDP include/arch/linux/udp_socket.c)

# Can manually add the sources using the set command as follows:
set(SOURCES src/user_ai.cpp src/natnet.cpp)

# However, the file(GLOB...) allows for wildcard additions:
# file(GLOB SOURCES "./*.c")

add_executable(natnet-fwd ${SOURCES})

# CMAKE_THREAD_LIBS_INIT = pthread, and for time, append "rt"
target_link_libraries(natnet-fwd UDP ${CMAKE_THREAD_LIBS_INIT})

# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Debug)
# endif()

