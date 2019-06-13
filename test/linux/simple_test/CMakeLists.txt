
set(SOURCES simple_test.c)
add_executable(simple_test ${SOURCES})
target_link_libraries(simple_test soem)
install(TARGETS simple_test DESTINATION bin)

set(SOURCES my_simple_test.c)
add_executable(my_simple_test ${SOURCES})
target_link_libraries(my_simple_test soem)
install(TARGETS my_simple_test DESTINATION bin)
