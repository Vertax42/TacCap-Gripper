find_package(GTest CONFIG REQUIRED)

add_executable(taccap_unit_tests
    test_crc.cpp
    test_frame.cpp
    test_codec.cpp
    test_codec_v16.cpp
    test_python_compat.cpp
    test_stuff.cpp
    test_transport.cpp
    test_components.cpp
    test_upper_v16.cpp
    test_upper_v16_pty.cpp
    test_crc32_boundaries.cpp
)

target_link_libraries(taccap_unit_tests
    PRIVATE
        taccap_core
        GTest::gtest
        GTest::gtest_main
        util            # openpty(), forkpty()
)

include(GoogleTest)
gtest_discover_tests(taccap_unit_tests)
