find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_taccap_native MODULE bindings/module.cpp)
target_link_libraries(_taccap_native PRIVATE taccap_core)

# In-tree development build: place the .so next to the Python sources so
# `pip install -e .` and direct `python -c 'import xense.taccap'` from the
# repo root both work. scikit-build-core also picks this up via the install()
# rules below for wheel builds.
set_target_properties(_taccap_native PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/xense/taccap
    # Find libtaccap_core.so in dev (build dir) and in the installed wheel
    # ($ORIGIN = the directory the .so is loaded from).
    BUILD_RPATH "$<TARGET_FILE_DIR:taccap_core>"
    INSTALL_RPATH "$ORIGIN"
)

# Also drop libtaccap_core.so + libxensesdk.so beside the Python ext during
# development so editable installs can find them without LD_LIBRARY_PATH.
add_custom_command(TARGET _taccap_native POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:taccap_core>
            ${CMAKE_CURRENT_SOURCE_DIR}/xense/taccap/$<TARGET_FILE_NAME:taccap_core>
    COMMAND ${CMAKE_COMMAND} -E create_symlink
            $<TARGET_FILE_NAME:taccap_core>
            ${CMAKE_CURRENT_SOURCE_DIR}/xense/taccap/$<TARGET_SONAME_FILE_NAME:taccap_core>
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<TARGET_FILE:xensesdk>
            ${CMAKE_CURRENT_SOURCE_DIR}/xense/taccap/$<TARGET_FILE_NAME:xensesdk>
    COMMAND ${CMAKE_COMMAND} -E create_symlink
            $<TARGET_FILE_NAME:xensesdk>
            ${CMAKE_CURRENT_SOURCE_DIR}/xense/taccap/$<TARGET_SONAME_FILE_NAME:xensesdk>
    VERBATIM)

# Wheel: ship _taccap_native + libtaccap_core.so + libxensesdk.so together.
install(TARGETS _taccap_native taccap_core xensesdk
    LIBRARY DESTINATION xense/taccap)
