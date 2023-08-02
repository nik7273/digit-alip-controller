#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include "Digit_Controller.hpp"
#include "lowlevelapi.h"

namespace py = pybind11;

PYBIND11_MODULE(digit_controller_pybind, m) {
    m.doc() = "pybind11 digit_controller_pybind plugin"; // optional module docstring

    // digit controller
    py::class_<Digit_Controller>(m, "Digit_Controller")
        .def(py::init<>())
        .def("Initialize_", &Digit_Controller::Initialize_)
        .def("Set_Initial_Standing_Gains_", &Digit_Controller::Set_Initial_Standing_Gains_)
        .def("Set_Initial_Walking_Gains_", &Digit_Controller::Set_Initial_Walking_Gains_)
        .def("Update_", &Digit_Controller::Update_);

    // agility llapi types
    py::class_<llapi_quaternion_t>(m, "Quaternion")
        .def(py::init<>())
        .def_readwrite("w", &llapi_quaternion_t::w)
        .def_readwrite("x", &llapi_quaternion_t::x)
        .def_readwrite("y", &llapi_quaternion_t::y)
        .def_readwrite("z", &llapi_quaternion_t::z);

    py::class_<llapi_observation_t>(m, "Observation")
        .def(py::init<>())
        .def_readwrite("time", &llapi_observation_t::time)
        .def_readwrite("error", &llapi_observation_t::error)
        .def_readwrite("base", &llapi_observation_t::base)
        .def_readwrite("imu", &llapi_observation_t::imu)
        .def_readwrite("motor", &llapi_observation_t::motor)
        .def_readwrite("joint", &llapi_observation_t::joint)
        .def_readwrite("battery_charge", &llapi_observation_t::battery_charge);

    // llapi_observation_t subtypes
    py::class_<llapi_observation_t::Base>(m, "Base")
        .def(py::init<>())
        .def_readwrite("translation", &llapi_observation_t::Base::translation)
        .def_readwrite("orientation", &llapi_observation_t::Base::orientation)
        .def_readwrite("linear_velocity", &llapi_observation_t::Base::linear_velocity)
        .def_readwrite("angular_velocity", &llapi_observation_t::Base::angular_velocity);

    py::class_<llapi_observation_t::IMU>(m, "IMU")
        .def(py::init<>())
        .def_readwrite("orientation", &llapi_observation_t::IMU::orientation)
        .def_readwrite("angular_velocity", &llapi_observation_t::IMU::angular_velocity)
        .def_readwrite("linear_acceleration", &llapi_observation_t::IMU::linear_acceleration)
        .def_readwrite("magnetic_field", &llapi_observation_t::IMU::magnetic_field);

    py::class_<llapi_observation_t::Motor>(m, "Motor")
        .def(py::init<>())
        .def_readwrite("position", &llapi_observation_t::Motor::position)
        .def_readwrite("velocity", &llapi_observation_t::Motor::velocity)
        .def_readwrite("torque", &llapi_observation_t::Motor::torque)

    py::class_<llapi_observation_t::Joint>(m, "Joint")
        .def(py::init<>())
        .def_readwrite("position", &llapi_observation_t::Joint::position)
        .def_readwrite("velocity", &llapi_observation_t::Joint::velocity)

    py::class_<llapi_limits_t>(m, "Limits")
        .def(py::init<>())
        .def_property("torque_limit", [](const llapi_limits_t &self) -> py::array {
            // getter
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {NUM_MOTORS}, {sizeof(double)});
            return py::array(dtype, {NUM_MOTORS}, {sizeof(double)}, self.torque_limit, base);
        }, [](llapi_limits_t &self, py::array_t<double> arr) {
            // setter
            std::memcpy(self.torque_limit, arr.data(), NUM_MOTORS * sizeof(double));
        })
        .def_property("damping_limit", [](const llapi_limits_t &self) -> py::array {            
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {NUM_MOTORS}, {sizeof(double)});
            return py::array(dtype, {NUM_MOTORS}, {sizeof(double)}, self.damping_limit, base);
        }, [](llapi_limits_t &self, py::array_t<double> arr) {
            std::memcpy(self.damping_limit, arr.data(), NUM_MOTORS * sizeof(double));
        })
        .def_property("velocity_limit", [](const llapi_limits_t &self) -> py::array {
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {NUM_MOTORS}, {sizeof(double)});
            return py::array(dtype, {NUM_MOTORS}, {sizeof(double)}, self.velocity_limit, base);
        }, [](llapi_limits_t &self, py::array_t<double> arr) {
            std::memcpy(self.velocity_limit, arr.data(), NUM_MOTORS * sizeof(double));
        });

    py::class_<llapi_motor_t>(m, "Motor")
        .def(py::init<>())
        .def_readwrite("torque", &llapi_motor_t::torque)
        .def_readwrite("velocity", &llapi_motor_t::velocity)
        .def_readwrite("damping", &llapi_motor_t::damping);

    py::class_<llapi_command_t>(m, "Command")
        .def(py::init<>())
        .def_property("motors", [](const llapi_command_t &self) -> py::array {
            auto dtype = py::dtype(py::format_descriptor<double>::format());
            auto base = py::array(dtype, {NUM_MOTORS}, {sizeof(double)});
            return py::array(dtype, {NUM_MOTORS}, {sizeof(double)}, self.motors, base);
        }, [](llapi_command_t &self, py::array_t<double> arr) {
            std::memcpy(self.motors, arr.data(), NUM_MOTORS * sizeof(double));
        })
        .def_readwrite("fallback_opmode", &llapi_command_t::fallback_opmode)
        .def_readwrite("apply_command", &llapi_command_t::apply_command);
}