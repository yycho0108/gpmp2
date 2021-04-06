#include <fstream>
#include <iostream>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

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
      .def_readwrite("radius", &PlanarGPMP2Settings::radius)
      .def_readwrite("verbosity", &PlanarGPMP2Settings::verbosity);

  // Add GPMP2Interface class.
  py::class_<PlanarGPMP2>(m, "PlanarGPMP2")
      .def(py::init<const PlanarGPMP2Settings&>())
      .def("SetOpts", &PlanarGPMP2::SetOpts)
      // .def("Init", &PlanarGPMP2::Init)
      .def("Init", [](PlanarGPMP2& self, py::array_t<std::uint8_t, py::array::c_style | py::array::forcecast> img_buf, const float cell_size){
              const int rows = img_buf.shape(0);
              const int cols = img_buf.shape(1);

              // NOTE(ycho): Not the most satisfactory resolution.
              const cv::Mat img(rows, cols, CV_8UC1, const_cast<std::uint8_t*>(img_buf.data()));
              return self.Init(img, cell_size);
              })
      .def("Plan", &PlanarGPMP2::Plan);
}
