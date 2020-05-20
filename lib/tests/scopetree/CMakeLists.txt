################################################
#   
################################################

link_directories(${CPPUTEST_LIBRARIES})

include_directories(../)

add_executable(nablaScopeTreeTests
        ${SOURCES}
        scopeTreeTests.cpp
        main.cpp
)

target_link_libraries(nablaScopeTreeTests 
                      ${CPPUTEST_LDFLAGS})

add_custom_command(TARGET nablaScopeTreeTests COMMAND ./nablaScopeTreeTests POST_BUILD)