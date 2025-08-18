#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "rby1-sdk/math/trapezoidal_motion_generator.h"

namespace py = pybind11;
using namespace rb;
using namespace py::literals;

template <int N>
void bind_trapezoidal_motion_generator(py::module_& m) {
  std::stringstream ss;
  ss << "TrapezoidalMotionGenerator";
  if constexpr (N > 0) {
    ss << "_" << N;
  }

  py::class_<rb::TrapezoidalMotionGenerator<N>> gen(m, ss.str().c_str());

  py::class_<typename rb::TrapezoidalMotionGenerator<N>::Input>(gen, "Input")
      .def(py::init<>())
      .def_readwrite("current_position", &rb::TrapezoidalMotionGenerator<N>::Input::current_position)
      .def_readwrite("current_velocity", &rb::TrapezoidalMotionGenerator<N>::Input::current_velocity)
      .def_readwrite("target_position", &rb::TrapezoidalMotionGenerator<N>::Input::target_position)
      .def_readwrite("velocity_limit", &rb::TrapezoidalMotionGenerator<N>::Input::velocity_limit)
      .def_readwrite("acceleration_limit", &rb::TrapezoidalMotionGenerator<N>::Input::acceleration_limit)
      .def_readwrite("minimum_time", &rb::TrapezoidalMotionGenerator<N>::Input::minimum_time);

  py::class_<typename rb::TrapezoidalMotionGenerator<N>::Output>(gen, "Output")
      .def(py::init<>())
      .def_readwrite("position", &rb::TrapezoidalMotionGenerator<N>::Output::position)
      .def_readwrite("velocity", &rb::TrapezoidalMotionGenerator<N>::Output::velocity)
      .def_readwrite("acceleration", &rb::TrapezoidalMotionGenerator<N>::Output::acceleration);

  py::class_<typename rb::TrapezoidalMotionGenerator<N>::Coeff>(gen, "Coeff")
      .def(py::init<>())
      .def_readwrite("start_t", &rb::TrapezoidalMotionGenerator<N>::Coeff::start_t)
      .def_readwrite("end_t", &rb::TrapezoidalMotionGenerator<N>::Coeff::end_t)
      .def_readwrite("init_p", &rb::TrapezoidalMotionGenerator<N>::Coeff::init_p)
      .def_readwrite("init_v", &rb::TrapezoidalMotionGenerator<N>::Coeff::init_v)
      .def_readwrite("a", &rb::TrapezoidalMotionGenerator<N>::Coeff::a);

  gen.def(py::init<unsigned int>(), "max_iter"_a = 30)
      .def("update", &rb::TrapezoidalMotionGenerator<N>::Update, "input"_a)
      .def("get_total_time", &rb::TrapezoidalMotionGenerator<N>::GetTotalTime)
      .def("__call__", &rb::TrapezoidalMotionGenerator<N>::operator())
      .def("at_time", &rb::TrapezoidalMotionGenerator<N>::at_time, "t"_a);
}

void pybind11_math(py::module_& m) {
  bind_trapezoidal_motion_generator<-1>(m);
}
