#include <fstream>
#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "interface.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pygpmp2, m) {
  m.doc() = "gpmp2 python plugin";

  // Add Settings class.
  py::class_<PlanarGPMP2Settings>(m, "PlanarGPMP2Settings")
      .def(py::init<>())
      .def_readwrite("total_time_sec", &PlanarGPMP2Settings::total_time_sec)
      .def_readwrite("total_time_step", &PlanarGPMP2Settings::total_time_step)
      .def_readwrite("check_inter", &PlanarGPMP2Settings::check_inter)
      .def_readwrite("use_vehicle_dynamics",
                     &PlanarGPMP2Settings::use_vehicle_dynamics)
      .def_readwrite("dynamics_sigma", &PlanarGPMP2Settings::dynamics_sigma)
      .def_readwrite("cost_sigma", &PlanarGPMP2Settings::cost_sigma)
      .def_readwrite("gp_sigma", &PlanarGPMP2Settings::gp_sigma)
      .def_readwrite("epsilon_dist", &PlanarGPMP2Settings::epsilon_dist)
      .def_readwrite("radius", &PlanarGPMP2Settings::radius);

  // Add GPMP2Interface class.
  py::class_<PlanarGPMP2>(m, "PlanarGPMP2")
      .def(py::init<const PlanarGPMP2Settings&>())
      .def("SetOpts", &PlanarGPMP2::SetOpts)
      .def("Init", &PlanarGPMP2::Init)
      .def("Plan", &PlanarGPMP2::Plan);
}
