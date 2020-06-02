################################################
#   Solace
################################################

link_directories(${CPPUTEST_LIBRARIES})

include_directories(../)

add_executable(nablaUtilityTests
        ${SOURCES}
        utilTests.cpp
        main.cpp
)

message(${CPPUTEST_LDFLAGS})

target_link_libraries(nablaUtilityTests 
                      ${CPPUTEST_LDFLAGS})

add_custom_command(TARGET nablaUtilityTests COMMAND ./nablaUtilityTests POST_BUILD)