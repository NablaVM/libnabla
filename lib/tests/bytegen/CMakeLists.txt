################################################
#   Solace
################################################

link_directories(${CPPUTEST_LIBRARIES})

include_directories(../)

add_executable(nablaBytegenTests
        ${SOURCES}
        arithmaticTests.cpp
        bitwiseTests.cpp
        branchTests.cpp
        constantTests.cpp
        callTests.cpp
        pcallTests.cpp
        exitTests.cpp
        jumpTests.cpp
        movTests.cpp
        nopTests.cpp
        pushpopTests.cpp
        returnTests.cpp
        sizeTests.cpp
        stbldbTests.cpp
        yieldTests.cpp
        main.cpp
)

target_link_libraries(nablaBytegenTests 
                      ${CPPUTEST_LDFLAGS})

add_custom_command(TARGET nablaBytegenTests COMMAND ./nablaBytegenTests POST_BUILD)