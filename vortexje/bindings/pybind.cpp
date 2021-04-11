#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <vortexje/bindings/case.hpp>


namespace py = pybind11;
using namespace py::literals;


PYBIND11_MODULE(euklid, m) {
    m.doc() = "vortexje panel method";
    //m.attr("__version__") = py::str(Vortexje::version);
}