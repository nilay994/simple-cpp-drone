cmake_minimum_required(VERSION 2.8.9)
project(rpi-msp-uart)

# find_package(Threads REQUIRED)

# Bring the headers
# include_directories(arch/linux)

# Can manually add the sources using the set command as follows:
set(SOURCES msp_node.cpp serial.cpp)

# However, the file(GLOB...) allows for wildcard additions:
# file(GLOB SOURCES "./*.c")

add_executable(rpi-msp-uart ${SOURCES})

# required for forcing -lpthread
# target_link_libraries(state_machine_example ${CMAKE_THREAD_LIBS_INIT} ${Boost_LIBRARIES} -lrt)
