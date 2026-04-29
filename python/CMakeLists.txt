find_package(pybind11 CONFIG REQUIRED)

pybind11_add_module(_taccap_native MODULE bindings/module.cpp)
target_link_libraries(_taccap_native PRIVATE taccap_core)

# In-tree development build: place the .so next to the Python sources so
# `pip install -e .` and direct `python -c 'import xense.taccap'` from the
# repo root both work. scikit-build-core also picks this up via the install()
# rule below for wheel builds.
set_target_properties(_taccap_native PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/xense/taccap
)

install(TARGETS _taccap_native LIBRARY DESTINATION xense/taccap)
