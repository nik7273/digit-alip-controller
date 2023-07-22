#include <pybind11/pybind11.h>
#include "Digit_Controller.hpp"

namespace py = pybind11;

PYBIND11_MODULE(digit_controller_pybind, m) {
    m.doc() = "pybind11 digit_controller_pybind plugin"; // optional module docstring

    py::class_<Digit_Controller>(m, "Digit_Controller")
        .def(py::init<>())
        .def("Initialize_", &Digit_Controller::Initialize_)
        .def("Set_Initial_Standing_Gains_", &Digit_Controller::Set_Initial_Standing_Gains_)
        .def("Set_Initial_Walking_Gains_", &Digit_Controller::Set_Initial_Walking_Gains_)
        .def("Update_", &Digit_Controller::Update_);
}