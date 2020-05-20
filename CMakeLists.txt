cmake_minimum_required(VERSION 3.8)

project(libNabla)

option(COMPILE_TESTS        "Execute compile-time tests"     ON)

include(${CMAKE_SOURCE_DIR}/cmake/FindCppuTest.cmake)

include(${CMAKE_SOURCE_DIR}/cmake/SetEnv.cmake)

set(CMAKE_CXX_STANDARD 17)

##################################################
# Set include directories
##################################################

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/bytegen)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/sockets)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/util)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/json)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/projectfs)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/scopetree)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/vsys)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/vsys/hardware)

##################################################
# Add Library Headers / Sources
##################################################

set(HEADERS
    bytegen/bytegen.hpp

    util/util.h
    sockets/sockets.h
    sockets/sockpool.h

    json/json.hpp

    scopetree/scopetree.hpp

    projectfs/projectfs.hpp
    projectfs/projecttype.hpp
    projectfs/projectstructure.hpp

    vsys/VSysExecutionContext.hpp
    vsys/VSysExecutionReturns.hpp
    vsys/VSysMachine.hpp
    vsys/VSysMemory.hpp
    vsys/VSysSettings.hpp
    vsys/VSysLoadableMachine.hpp
    vsys/hardware/VSysExternalIO.hpp
    vsys/hardware/VSysExternalNet.hpp
    vsys/hardware/VSysExternalHost.hpp
)

set(SOURCES
    bytegen/bytegen.cpp

    util/util.c
    sockets/sockets.c
    sockets/sockpool.c

    scopetree/scopetree.cpp
    projectfs/projectfs.cpp
    projectfs/projectstructure.cpp
    
    vsys/VSysExecutionContext.cpp
    vsys/VSysMachine.cpp
    vsys/VSysLoadableMachine.cpp
    vsys/hardware/VSysExternalIO.cpp
    vsys/hardware/VSysExternalNet.cpp
    vsys/hardware/VSysExternalHost.cpp
)

##################################################
# Configure Library
##################################################

include(${CMAKE_SOURCE_DIR}/cmake/LibraryConfig.cmake)

##################################################
# Tests
##################################################

if(COMPILE_TESTS)
        add_subdirectory(tests)
endif()

##################################################
# Configure Install
##################################################

include(${CMAKE_SOURCE_DIR}/cmake/InstallConfig.cmake)