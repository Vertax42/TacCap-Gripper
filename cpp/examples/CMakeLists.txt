add_executable(taccap_hello taccap_hello.cpp)
target_link_libraries(taccap_hello PRIVATE taccap_core)

add_executable(leader_demo leader_demo.cpp)
target_link_libraries(leader_demo PRIVATE taccap_core)
