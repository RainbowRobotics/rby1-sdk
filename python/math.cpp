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

  py::class_<rb::TrapezoidalMotionGenerator<N>> gen(m, ss.str().c_str(), R"doc(
Trapezoidal motion generator for smooth trajectory planning.

This class generates smooth trapezoidal velocity profiles for multi-joint
robot motion, ensuring velocity and acceleration limits are respected.

Parameters
----------
max_iter : int, optional
    Maximum number of iterations for optimization. Default is 30.

Attributes
----------
Input : class
    Input parameters for motion generation.
Output : class
    Output trajectory data.
Coeff : class
    Spline coefficients for trajectory segments.
)doc");

  py::class_<typename rb::TrapezoidalMotionGenerator<N>::Input>(gen, "Input", R"doc(
Input parameters for trapezoidal motion generation.

Attributes
----------
current_position : numpy.ndarray
    Current joint positions, shape (N,), dtype=float64.
current_velocity : numpy.ndarray
    Current joint velocities, shape (N,), dtype=float64.
target_position : numpy.ndarray
    Target joint positions, shape (N,), dtype=float64.
velocity_limit : numpy.ndarray
    Maximum allowed velocities for each joint, shape (N,), dtype=float64.
acceleration_limit : numpy.ndarray
    Maximum allowed accelerations for each joint, shape (N,), dtype=float64.
minimum_time : float
    Minimum time constraint for the motion. This parameter provides an additional degree
    of freedom to control the arrival time to a target. Instead of relying
    solely on velocity/acceleration limits, you can set high limits and
    control the arrival time using minimum_time. For streaming commands,
    this helps ensure continuous motion by preventing the robot from
    stopping if it arrives too early before the next command.
)doc")
      .def(py::init<>(), R"doc(
Construct an Input instance with default values.
)doc")
      .def_readwrite("current_position", &rb::TrapezoidalMotionGenerator<N>::Input::current_position, R"doc(
Current joint positions.

Type
----
numpy.ndarray
    Shape (N,), dtype=float64.
)doc")
      .def_readwrite("current_velocity", &rb::TrapezoidalMotionGenerator<N>::Input::current_velocity, R"doc(
Current joint velocities.

Type
----
numpy.ndarray
    Shape (N,), dtype=float64.
)doc")
      .def_readwrite("target_position", &rb::TrapezoidalMotionGenerator<N>::Input::target_position, R"doc(
Target joint positions.

Type
----
numpy.ndarray
    Shape (N,), dtype=float64.
)doc")
      .def_readwrite("velocity_limit", &rb::TrapezoidalMotionGenerator<N>::Input::velocity_limit, R"doc(
Maximum allowed velocities for each joint.

Type
----
numpy.ndarray
    Shape (N,), dtype=float64.
)doc")
      .def_readwrite("acceleration_limit", &rb::TrapezoidalMotionGenerator<N>::Input::acceleration_limit, R"doc(
Maximum allowed accelerations for each joint.

Type
----
numpy.ndarray
    Shape (N,), dtype=float64.
)doc")
      .def_readwrite("minimum_time", &rb::TrapezoidalMotionGenerator<N>::Input::minimum_time, R"doc(
Minimum time constraint for the motion. This parameter provides an additional degree
of freedom to control the arrival time to a target. Instead of relying
solely on velocity/acceleration limits, you can set high limits and
control the arrival time using minimum_time. For streaming commands,
this helps ensure continuous motion by preventing the robot from
stopping if it arrives too early before the next command.

Type
----
float
    Time in seconds.
)doc");

  py::class_<typename rb::TrapezoidalMotionGenerator<N>::Output>(gen, "Output", R"doc(
Output trajectory data from motion generation.

Attributes
----------
position : numpy.ndarray
    Joint positions at the specified time, shape (N,), dtype=float64.
velocity : numpy.ndarray
    Joint velocities at the specified time, shape (N,), dtype=float64.
acceleration : numpy.ndarray
    Joint accelerations at the specified time, shape (N,), dtype=float64.
)doc")
      .def(py::init<>(), R"doc(
Construct an Output instance with default values.
)doc")
      .def_readwrite("position", &rb::TrapezoidalMotionGenerator<N>::Output::position, R"doc(
Joint positions at the specified time.

Type
----
numpy.ndarray
    Shape (N,), dtype=float64.
)doc")
      .def_readwrite("velocity", &rb::TrapezoidalMotionGenerator<N>::Output::velocity, R"doc(
Joint velocities at the specified time.

Type
----
numpy.ndarray
    Shape (N,), dtype=float64.
)doc")
      .def_readwrite("acceleration", &rb::TrapezoidalMotionGenerator<N>::Output::acceleration, R"doc(
Joint accelerations at the specified time.

Type
----
numpy.ndarray
    Shape (N,), dtype=float64.
)doc");

  py::class_<typename rb::TrapezoidalMotionGenerator<N>::Coeff>(gen, "Coeff", R"doc(
Spline coefficients for trajectory segments.

Attributes
----------
start_t : float
    Start time of the segment.
end_t : float
    End time of the segment.
init_p : float
    Initial position for the segment.
init_v : float
    Initial velocity for the segment.
a : float
    Constant acceleration for the segment.
)doc")
      .def(py::init<>(), R"doc(
Construct a Coeff instance with default values.
)doc")
      .def_readwrite("start_t", &rb::TrapezoidalMotionGenerator<N>::Coeff::start_t, R"doc(
Start time of the segment.

Type
----
float
    Time in seconds.
)doc")
      .def_readwrite("end_t", &rb::TrapezoidalMotionGenerator<N>::Coeff::end_t, R"doc(
End time of the segment.

Type
----
float
    Time in seconds.
)doc")
      .def_readwrite("init_p", &rb::TrapezoidalMotionGenerator<N>::Coeff::init_p, R"doc(
Initial position for the segment.

Type
----
float
    Position in radians or meters.
)doc")
      .def_readwrite("init_v", &rb::TrapezoidalMotionGenerator<N>::Coeff::init_v, R"doc(
Initial velocity for the segment.

Type
----
float
    Velocity in rad/s or m/s.
)doc")
      .def_readwrite("a", &rb::TrapezoidalMotionGenerator<N>::Coeff::a, R"doc(
Constant acceleration for the segment.

Type
----
float
    Acceleration in rad/s² or m/s².
)doc");

  gen.def(py::init<unsigned int>(), "max_iter"_a = 30, R"doc(
Construct a TrapezoidalMotionGenerator.

Parameters
----------
max_iter : int, optional
    Maximum number of iterations for optimization. Default is 30.
)doc")
      .def("update", &rb::TrapezoidalMotionGenerator<N>::Update, "input"_a, R"doc(
update(input)

Update the motion generator with new input parameters.

Parameters
----------
input : Input
    Input parameters for motion generation.

Raises
------
ValueError
    If input argument sizes are inconsistent.
)doc")
      .def("get_total_time", &rb::TrapezoidalMotionGenerator<N>::GetTotalTime, R"doc(
get_total_time()

Get the total time for the generated trajectory.

Returns
-------
float
    Total trajectory time in seconds.
)doc")
      .def("__call__", &rb::TrapezoidalMotionGenerator<N>::operator(), R"doc(
Get trajectory output at the specified time.

Parameters
----------
t : float
    Time at which to evaluate the trajectory.

Returns
-------
Output
    Trajectory data at time t.

Raises
------
RuntimeError
    If the motion generator is not initialized.
)doc")
      .def("at_time", &rb::TrapezoidalMotionGenerator<N>::at_time, "t"_a, R"doc(
at_time(t)

Get trajectory output at the specified time.

Parameters
----------
t : float
    Time at which to evaluate the trajectory.

Returns
-------
Output
    Trajectory data at time t.

Raises
------
RuntimeError
    If the motion generator is not initialized.
)doc");
}

void pybind11_math(py::module_& m) {
  bind_trapezoidal_motion_generator<-1>(m);
}
