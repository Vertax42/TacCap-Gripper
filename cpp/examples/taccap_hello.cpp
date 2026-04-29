// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// Smoke test: print TacCap and libxense lite version strings, confirm that
// the namespace alias compiles and the libxensesdk symbols link.

#include <iostream>

#include <taccap/version.hpp>
#include <taccap/vision.hpp>

#include <xense/core/version.hpp>  // defines XENSESDK_VERSION_STRING

int main() {
    std::cout << "taccap-gripper " << TACCAP_VERSION_STRING
              << " (libxense lite " << XENSESDK_VERSION_STRING << ")\n";

    // Touch the alias to ensure the using-declaration compiles end-to-end.
    auto ctx = xense::taccap::Context::create();
    std::cout << "xense::taccap::Context created OK\n";

    return 0;
}
