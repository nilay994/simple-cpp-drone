cmake_minimum_required(VERSION 3.1.0)
project(rpi-control)

add_subdirectory(include/qpOASES)

include_directories(include/qpOASES/include)

# set (CMAKE_CXX_STANDARD 11) # in CMAKE 3.1.0+

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "-Wall -Wextra")

set(CMAKE_CONFIGURATION_TYPES "Release" CACHE STRING "" FORCE)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Threads REQUIRED)

# include Eigen https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
find_package (Eigen3 3.3 NO_MODULE)

# MESSAGE( [Main] " EIGEN3_INCLUDE_DIRS = ${EIGEN3_INCLUDE_DIRS}") 
# EIGEN3_INCLUDE_DIRS = /usr/include/eigen3

# include_directories(${EIGEN3_INCLUDE_DIRS})

# Bring the headers
include_directories(include/arch/linux)
include_directories(include)

# include_directories(include/qpOASES/include)

    
# <include name="$(PAPARAZZI_SRC)/sw/airborne/modules/qpoas/"/>
# <include name="$(PAPARAZZI_SRC)/sw/airborne/modules/qpoas/qpoases"/>
# <include name="$(PAPARAZZI_SRC)/sw/airborne/modules/qpoas/qpoases/INCLUDE/qpOASES/"/>
# <include name="$(PAPARAZZI_SRC)/sw/airborne/modules/qpoas/qpoases/INCLUDE/"/>
# <include name="$(PAPARAZZI_SRC)/sw/airborne/modules/qpoas/qpoases/SRC/"/>)

# compile the libraries for linux architecture like udp socket
add_library(UDP include/arch/linux/udp_socket.c)

add_library(SERIAL src/serial.cpp)

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

# Bounds.cpp include/qpOASES/src/Constraints.cpp)

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



# <configure name="CXXSTANDARD" value="-std=c++14"/>
# <include name="$(PAPARAZZI_SRC)/sw/ext/eigen"/>
# <flag name="LDFLAGS" value="lstdc++ -lm -lrt" />
# <flag name="CXXFLAGS" value="Wno-bool-compare"/>
# <flag name="CXXFLAGS" value="Wno-logical-not-parentheses"/>
# <flag name="CXXFLAGS" value="O3 -finline-functions"/>
# <flag name="__SUPPRESSANYOUTPUT__" value=""/>
# <define name="EIGEN_NO_MALLOC"/>
# <define name="EIGEN_NO_AUTOMATIC_RESIZING"/>
# <define name="EIGEN_NO_DEBUG"/>
# <include name="$(PAPARAZZI_SRC)/sw/airborne/modules/qpoas/"/>
# <include name="$(PAPARAZZI_SRC)/sw/airborne/modules/qpoas/qpoases"/>
# <include name="$(PAPARAZZI_SRC)/sw/airborne/modules/qpoas/qpoases/INCLUDE/qpOASES/"/>
# <include name="$(PAPARAZZI_SRC)/sw/airborne/modules/qpoas/qpoases/INCLUDE/"/>
# <include name="$(PAPARAZZI_SRC)/sw/airborne/modules/qpoas/qpoases/SRC/"/>
# <file name="qpoases/SRC/Bounds.cpp"/>
# <file name="qpoases/SRC/Constraints.cpp"/>
# <file name="qpoases/SRC/QProblem.cpp"/>
# <file name="qpoases/SRC/QProblemB.cpp"/>
# <!--file name="qpoases/SRC/CyclingManager.cpp"/-->
# <file name="qpoases/SRC/Indexlist.cpp"/>
# <file name="qpoases/SRC/MessageHandling.cpp"/>
# <file name="qpoases/SRC/SubjectTo.cpp"/>
# <file name="qpoases/SRC/Utils.cpp"/>
# <file name="qpoases/SRC/BLASReplacement.cpp"/>
# <file name="qpoases/SRC/LAPACKReplacement.cpp"/>
# <file name="qpoases/SRC/Matrices.cpp"/>
# <file name="qpoases/SRC/Flipper.cpp"/>    
# <file name="qpoases/SRC/Indexlist.cpp"/>
# <file name="qpoases/SRC/Options.cpp"/>
# <file name="qpoases/SRC/OQPinterface.cpp"/>
