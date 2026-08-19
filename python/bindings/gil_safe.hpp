// Copyright (c) 2026 XenseRobotics Co., Ltd. — Apache-2.0
//
// GIL discipline for C++ -> Python subscriber callbacks, in one place.
//
// Every binding that hands a Python callable to a C++ worker thread needs the
// same two things, and getting either wrong is a segfault rather than an
// error:
//
//   1. make_gil_safe_callback() to own the callable. The last reference dies
//      on whatever thread runs the owning lambda's destructor -- the transport
//      reader/dispatcher, the camera capture thread, or Transport::stop() on a
//      caller that released the GIL. py::function's destructor decrefs a
//      Python object, which segfaults without the GIL held.
//   2. call_into_python() to invoke it, which takes the GIL and swallows both
//      Python and C++ exceptions so nothing escapes into a noexcept worker.
//
// This header exists because these lived in components.cpp's anonymous
// namespace, module.cpp could not reach them, and its Transport.subscribe
// binding used a bare make_shared instead -- crashing on stop() for every
// caller that had a raw subscription open. Shared code, one invariant.

#pragma once

#include <pybind11/pybind11.h>

#include <memory>
#include <utility>

namespace xense::taccap::python::gil {

namespace py = pybind11;

// True once the interpreter is finalizing or already gone.
//
// Worker threads (the transport reader, the camera capture thread) are plain
// std::threads: nothing stops them at interpreter shutdown, and a Transport
// that the user never stopped explicitly is often still reading when
// Py_Finalize runs. Touching Python from such a thread afterwards aborts the
// process inside PyGILState_Ensure — the "FATAL: exception not rethrown" you
// get when a late DATA frame lands during teardown. (The firmware keeps
// flushing queued frames for a while after StopStream, so this is easy to
// hit in practice, not a theoretical race.)
//
// _Py_IsFinalizing() is private-but-stable across CPython 3.7-3.12 and became
// public Py_IsFinalizing() in 3.13.
inline bool interpreter_gone() noexcept {
#if PY_VERSION_HEX >= 0x030D0000
    return Py_IsFinalizing() != 0 || Py_IsInitialized() == 0;
#else
    return _Py_IsFinalizing() != 0 || Py_IsInitialized() == 0;
#endif
}

// Wrap a py::function in a shared_ptr whose deleter acquires the GIL.
// Background: the per-component callback wrappers below capture the
// shared_ptr into the C++ callback lambda. The last shared_ptr ref dies
// on whatever thread runs the lambda's destructor — usually the worker
// capture thread when stop() joins it. py::function's destructor decrefs
// a Python object, which segfaults without the GIL. Centralising the
// custom deleter here keeps every component honest.
inline std::shared_ptr<py::function> make_gil_safe_callback(py::function pycb) {
    return std::shared_ptr<py::function>(
        new py::function(std::move(pycb)),
        [](py::function* p) {
            if (interpreter_gone()) {
                // Deliberately leak: the interpreter owns this object's
                // memory and is tearing down anyway. Decref'ing now would
                // abort the process.
                return;
            }
            py::gil_scoped_acquire gil;
            delete p;
        });
}

// Invoke a Python subscriber callback from a C++ worker thread. Skips the
// call entirely once the interpreter is gone — dropping a late event is the
// only safe option at that point. Exceptions never escape into the worker.
template <typename Fn>
void call_into_python(const char* what, Fn&& fn) noexcept {
    if (interpreter_gone()) return;
    py::gil_scoped_acquire acq;
    try {
        fn();
    } catch (py::error_already_set& e) {
        e.discard_as_unraisable(what);
    } catch (...) {
    }
}

}  // namespace xense::taccap::python::gil
