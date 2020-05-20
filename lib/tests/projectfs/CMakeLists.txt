################################################
#   
################################################

link_directories(${CPPUTEST_LIBRARIES})

include_directories(../)

add_executable(nablaProjectFsTests
        ${SOURCES}
        projectfsTests.cpp
        main.cpp
)

target_link_libraries(nablaProjectFsTests 
                      ${CPPUTEST_LDFLAGS})

add_custom_command(TARGET nablaProjectFsTests COMMAND ./nablaProjectFsTests POST_BUILD)