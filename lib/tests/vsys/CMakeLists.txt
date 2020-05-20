################################################
#   
################################################

link_directories(${CPPUTEST_LIBRARIES})

include_directories(../)

add_executable(nablaVsysTests
        ${SOURCES}
        automatedExecutionTests.cpp
        arithTests.cpp
        bitwiseTests.cpp
        branchTests.cpp
        jumpTests.cpp
        memory.cpp
        movTests.cpp
        nopTests.cpp
        pushPopTests.cpp
        sizeTests.cpp
        main.cpp
)

target_link_libraries(nablaVsysTests 
                      ${CPPUTEST_LDFLAGS})

add_custom_command(TARGET nablaVsysTests COMMAND ./nablaVsysTests POST_BUILD)