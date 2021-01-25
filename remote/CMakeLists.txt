cmake_minimum_required(VERSION 3.1.0)
project(rpi-control)

# set (CMAKE_CXX_STANDARD 11) # in CMAKE 3.1.0+

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Threads REQUIRED)

# include Eigen https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
# /usr/include/eigen3
find_package (Eigen3 3.3 NO_MODULE)

# Bring the headers
include_directories(include/arch/linux)
include_directories(include)
include_directories(include/qpOASES/include)

# following subdirectory must have a CMakeLists,
# including this directory means compiling 
# its executables as well (not necessary)
# add_subdirectory(include/qpOASES)

# compile the libraries for linux architecture like udp socket
add_library(UDP include/arch/linux/udp_socket.c)

add_library(SERIAL src/serial.cpp)

# CMake guys don't advise globbing but want explicit mention of libs
# file(GLOB_RECURSE OPTIMIZER "include/qpOASES/src/*.cpp")

# trying to mute the unsused variable warnings while including lib
# file(GLOB_RECURSE SRC_DIR "include/qpOASES/src/*.cpp" "include/qpOASES/include/*.hpp")
# set_source_files_properties(${SRC_DIR} PROPERTIES COMPILE_FLAGS "-Wno-unused-variable")

add_library(OPTIMIZER
        include/qpOASES/src/QProblemB.cpp 
        include/qpOASES/src/BLASReplacement.cpp
        include/qpOASES/src/Constraints.cpp
        include/qpOASES/src/Indexlist.cpp
        include/qpOASES/src/Matrices.cpp
        include/qpOASES/src/Options.cpp
        include/qpOASES/src/QProblemB.cpp
        include/qpOASES/src/SolutionAnalysis.cpp
        include/qpOASES/src/SubjectTo.cpp
        include/qpOASES/src/Bounds.cpp
        include/qpOASES/src/Flipper.cpp
        include/qpOASES/src/LAPACKReplacement.cpp
        include/qpOASES/src/MessageHandling.cpp
        include/qpOASES/src/OQPinterface.cpp 
        include/qpOASES/src/QProblem.cpp
        include/qpOASES/src/SQProblem.cpp
        include/qpOASES/src/Utils.cpp)

# Can manually add the sources using the set command as follows:
set(SOURCES src/user_ai.cpp src/control.cpp src/natnet.cpp src/msp_node.cpp src/state_machine.cpp src/flightplan.cpp src/ctrl_optimizer.cpp)
# However, the file(GLOB...) allows for wildcard additions:
# file(GLOB SOURCES "./*.c")

add_executable(simple-cpp-drone ${SOURCES})

# CMAKE_THREAD_LIBS_INIT = pthread, and for time, append "rt"
target_link_libraries(simple-cpp-drone UDP SERIAL ${CMAKE_THREAD_LIBS_INIT} Eigen3::Eigen OPTIMIZER)

# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Debug)
# endif()