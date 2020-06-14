cmake_minimum_required(VERSION 3.8)

#-------------------------------------------------
# Project 
#-------------------------------------------------

project(libnabla)

#-------------------------------------------------
# Options
#-------------------------------------------------

option(COMPILE_TESTS        "Execute compile-time tests"     ON)
option(VM_DEBUG_OUTPUT      "Enable VM debug output"        OFF)

set(CMAKE_CXX_STANDARD 17)

if(VM_DEBUG_OUTPUT)
    add_definitions(-DNABLA_VIRTUAL_MACHINE_DEBUG_OUTPUT)
endif(VM_DEBUG_OUTPUT)

#-------------------------------------------------
# Env and Cpputest
#-------------------------------------------------

include(${CMAKE_SOURCE_DIR}/cmake/FindCppuTest.cmake)

include(${CMAKE_SOURCE_DIR}/cmake/SetEnv.cmake)

#-------------------------------------------------
# Set include directories
#-------------------------------------------------

include_directories(/usr/local/include)
include_directories(${CMAKE_SOURCE_DIR}/endian)
include_directories(${CMAKE_SOURCE_DIR}/assembler)
include_directories(${PROJECT_SOURCE_DIR}/bytegen)
include_directories(${CMAKE_SOURCE_DIR}/sockets)
include_directories(${CMAKE_SOURCE_DIR}/util)
include_directories(${CMAKE_SOURCE_DIR}/json)
include_directories(${CMAKE_SOURCE_DIR}/projectfs)
include_directories(${CMAKE_SOURCE_DIR}/termcolor)
include_directories(${CMAKE_SOURCE_DIR}/vsys)
include_directories(${CMAKE_SOURCE_DIR}/vsys/hardware)

#-------------------------------------------------
# Add Library Headers / Sources
#-------------------------------------------------

set(HEADERS
    ${CMAKE_SOURCE_DIR}/endian/endian.hpp
    ${CMAKE_SOURCE_DIR}/assembler/assembler.hpp
    ${CMAKE_SOURCE_DIR}/bytegen/bytegen.hpp
    ${CMAKE_SOURCE_DIR}/util/util.hpp
    ${CMAKE_SOURCE_DIR}/sockets/sockets.h
    ${CMAKE_SOURCE_DIR}/sockets/sockpool.h
    ${CMAKE_SOURCE_DIR}/json/json.hpp
    ${CMAKE_SOURCE_DIR}/projectfs/projectfs.hpp
    ${CMAKE_SOURCE_DIR}/projectfs/projecttype.hpp
    ${CMAKE_SOURCE_DIR}/projectfs/projectstructure.hpp
    ${CMAKE_SOURCE_DIR}/termcolor/termcolor.hpp
    ${CMAKE_SOURCE_DIR}/vsys/VSysInstructions.hpp
    ${CMAKE_SOURCE_DIR}/vsys/VSysExecutionContext.hpp
    ${CMAKE_SOURCE_DIR}/vsys/VSysExecutionReturns.hpp
    ${CMAKE_SOURCE_DIR}/vsys/VSysMachine.hpp
    ${CMAKE_SOURCE_DIR}/vsys/VSysMemory.hpp
    ${CMAKE_SOURCE_DIR}/vsys/VSysSettings.hpp
    ${CMAKE_SOURCE_DIR}/vsys/VSysLoadableMachine.hpp
    ${CMAKE_SOURCE_DIR}/vsys/hardware/VSysExternalIf.hpp
    ${CMAKE_SOURCE_DIR}/vsys/hardware/VSysExternalIO.hpp
    ${CMAKE_SOURCE_DIR}/vsys/hardware/VSysExternalNet.hpp
    ${CMAKE_SOURCE_DIR}/vsys/hardware/VSysExternalHost.hpp
)

set(SOURCES
    ${CMAKE_SOURCE_DIR}/assembler/assembler.cpp
    ${CMAKE_SOURCE_DIR}/bytegen/bytegen.cpp
    ${CMAKE_SOURCE_DIR}/util/util.cpp
    ${CMAKE_SOURCE_DIR}/sockets/sockets.c
    ${CMAKE_SOURCE_DIR}/sockets/sockpool.c
    ${CMAKE_SOURCE_DIR}/projectfs/projectfs.cpp
    ${CMAKE_SOURCE_DIR}/projectfs/projectstructure.cpp
    ${CMAKE_SOURCE_DIR}/vsys/VSysExecutionContext.cpp
    ${CMAKE_SOURCE_DIR}/vsys/VSysMachine.cpp
    ${CMAKE_SOURCE_DIR}/vsys/VSysLoadableMachine.cpp
    ${CMAKE_SOURCE_DIR}/vsys/hardware/VSysExternalIO.cpp
    ${CMAKE_SOURCE_DIR}/vsys/hardware/VSysExternalNet.cpp
    ${CMAKE_SOURCE_DIR}/vsys/hardware/VSysExternalHost.cpp
)

#-------------------------------------------------
# Configure Library
#-------------------------------------------------

include(${CMAKE_SOURCE_DIR}/cmake/LibraryConfig.cmake)

#-------------------------------------------------
# Tests
#-------------------------------------------------

if(COMPILE_TESTS)
        add_subdirectory(tests)
endif()

#-------------------------------------------------
# Configure Install
#-------------------------------------------------

include(${CMAKE_SOURCE_DIR}/cmake/InstallConfig.cmake)