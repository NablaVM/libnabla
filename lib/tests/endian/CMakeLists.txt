################################################
#   Solace
################################################

link_directories(${CPPUTEST_LIBRARIES})

include_directories(../)

add_executable(nablaEndianTests
        ${SOURCES}
        endian.cpp
        main.cpp
)

message(${CPPUTEST_LDFLAGS})

target_link_libraries(nablaEndianTests 
                      ${CPPUTEST_LDFLAGS})

add_custom_command(TARGET nablaEndianTests COMMAND ./nablaEndianTests POST_BUILD)