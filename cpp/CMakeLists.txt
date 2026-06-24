# cpp/ — TacCap-Gripper core library + C++ tests/examples

add_library(taccap_core SHARED
    src/protocol/commands.cpp
    src/protocol/codec.cpp
    src/bus/frame.cpp
    src/bus/serial_bus.cpp
    src/bus/transport.cpp
    src/components/imu.cpp
    src/components/encoder.cpp
    src/components/camera.cpp
    src/components/tactile_sensor.cpp
    src/components/motor.cpp
    src/components/key.cpp
    src/components/sensor_errors.cpp
    src/discovery.cpp
    src/leader_gripper.cpp
    src/follower_gripper.cpp
    src/control_loop.cpp
    src/ota.cpp
)

# Generate the version header into the build tree.
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/taccap/version.hpp.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/taccap/version.hpp
    @ONLY
)

target_include_directories(taccap_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
)

# Camera component uses cv::VideoCapture which lives in opencv_videoio
# (not in libxensesdk's transitive set). Pull in just that one extra
# component; the rest of OpenCV comes through xensesdk PUBLIC.
find_package(OpenCV REQUIRED COMPONENTS videoio)

target_link_libraries(taccap_core
    PUBLIC  xensesdk             # libxense lite — vision.hpp aliases
    PUBLIC  opencv_videoio       # cv::VideoCapture in Camera component
    PRIVATE Threads::Threads     # poll() + future async transport
)

set_target_properties(taccap_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    SOVERSION ${PROJECT_VERSION_MAJOR}
    VERSION   ${PROJECT_VERSION}
    OUTPUT_NAME taccap_core
    # In dev: find libxensesdk.so wherever cmake built it.
    # In wheel/install: $ORIGIN = same dir as libtaccap_core.so (we co-locate
    # libxensesdk.so there via the python install() rule).
    BUILD_RPATH   "$<TARGET_FILE_DIR:xensesdk>"
    INSTALL_RPATH "$ORIGIN"
)

# C++ examples (smoke executables, not shipped in the wheel).
if(TACCAP_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# C++ unit tests. enable_testing() is called at top level so ctest sees us.
if(TACCAP_BUILD_TESTS)
    add_subdirectory(tests)
endif()
