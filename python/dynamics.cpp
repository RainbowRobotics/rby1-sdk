#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <Eigen/Core>
#include <iomanip>
#include <optional>
#include <utility>

#include "common.h"
#include "print_helper.h"
#include "rby1-sdk/dynamics/robot.h"
#include "rby1-sdk/dynamics/state.h"
#include "rby1-sdk/model.h"

namespace py = pybind11;
using namespace rb;
using namespace py::literals;

class PyGeom : public dyn::Geom {
 public:
  using dyn::Geom::Geom;

  dyn::GeomType GetType() const override { PYBIND11_OVERRIDE_PURE(dyn::GeomType, dyn::Geom, GetType); }

  std::optional<dyn::CollisionResult> ComputeMinimumDistance(const math::SE3::MatrixType& T, const Geom& other_geom,
                                                             const math::SE3::MatrixType& other_T) const override {
    PYBIND11_OVERRIDE_PURE(std::optional<dyn::CollisionResult>, dyn::Geom, ComputeMinimumDistance, T, other_geom,
                           other_T);
  }

  using dyn::Geom::GetColaffinity;
  using dyn::Geom::GetColtype;
};

void bind_collision_result(py::module_& m) {
  py::class_<dyn::CollisionResult>(m, "CollisionResult", R"doc(
Collision detection result.

Provides information about collision detection between two geometric objects.

Attributes
----------
link1 : str
    Name of the first link involved in collision.
link2 : str
    Name of the second link involved in collision.
position1 : numpy.ndarray
    Position of collision point on first link in meters.
position2 : numpy.ndarray
    Position of collision point on second link in meters.
distance : float
    Signed distance in meters. Positive when separated, ``0`` when touching,
    negative when overlapping (penetration depth = ``-distance``).
  )doc")
      .def(py::init<>(), R"doc(
      Construct a ``CollisionResult`` instance.
)doc")
      .def_readonly("link1", &dyn::CollisionResult::link1, R"doc(
First link name.

Type
----
str
    Name of the first link involved in collision.
)doc")
      .def_readonly("link2", &dyn::CollisionResult::link2, R"doc(
Second link name.

Type
----
str
    Name of the second link involved in collision.
)doc")
      .def_readonly("position1", &dyn::CollisionResult::position1, R"doc(
First link collision position.

Type
----
numpy.ndarray
    Position of collision point on first link in meters.
)doc")
      .def_readonly("position2", &dyn::CollisionResult::position2, R"doc(
Second link collision position.

Type
----
numpy.ndarray
    Position of collision point on second link in meters.
)doc")
      .def_readonly("distance", &dyn::CollisionResult::distance, R"doc(
Minimum distance.

Type
----
float
    Minimum distance between the objects in meters.
)doc")
      .def("__repr__",
           [](const dyn::CollisionResult& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             std::ostringstream out;
             out << "CollisionResult(" << FIRST << "link1=" << inline_obj(py::cast(self.link1)) << SEP
                 << "link2=" << inline_obj(py::cast(self.link2)) << SEP
                 << "position1=" << np_array_to_string(py::cast(self.position1), Style::Repr) << SEP
                 << "position2=" << np_array_to_string(py::cast(self.position2), Style::Repr) << SEP
                 << "distance=" << format_number(self.distance, Style::Repr) << LAST << ")";
             return out.str();
           })
      .def("__str__", [](const dyn::CollisionResult& self) {
        using namespace rb::print;
        std::ostringstream out;
        out << "CollisionResult("
            << "l1=" << inline_obj_one_line(py::cast(self.link1)) << ", "
            << "l2=" << inline_obj_one_line(py::cast(self.link2)) << ", "
            << "d=" << format_number(self.distance, Style::Str) << ")";
        return out.str();
      });
}

void bind_geom(py::module_& m) {
  py::enum_<dyn::GeomType>(m, "GeomType", R"doc(
Geometry type enumeration.

Defines the types of geometric objects supported.

Members
-------
Capsule : int
    Capsule geometry type.
  )doc")
      .value("Capsule", dyn::GeomType::kCapsule, R"doc(
      Capsule geometry type.
)doc");

  py::class_<dyn::Geom, PyGeom, std::shared_ptr<dyn::Geom>>(m, "Geom", R"doc(
Base geometry class.

Abstract base class for geometric objects used in collision detection.

Attributes
----------
coltype : int
    Collision type identifier.
colaffinity : int
    Collision affinity identifier.
  )doc")
      .def(py::init<unsigned int, unsigned int>(), R"doc(
Construct a ``Geom`` instance.

Parameters
----------
coltype : int
    Collision type identifier.
colaffinity : int
    Collision affinity identifier.
)doc")
      .def("get_type", &dyn::Geom::GetType, R"doc(
get_type()

Get the geometry type.

Returns
-------
GeomType
    Type of the geometry.
)doc")
      .def("get_coltype", &dyn::Geom::GetColtype, R"doc(
get_coltype()

Get the collision type.

Returns
-------
int
    Collision type identifier.
)doc")
      .def("get_colaffinity", &dyn::Geom::GetColaffinity, R"doc(
get_colaffinity()

Get the collision affinity.

Returns
-------
int
    Collision affinity identifier.
)doc")
      .def("compute_minimum_distance", &dyn::Geom::ComputeMinimumDistance, "T"_a, "other_geom"_a, "other_T"_a, R"doc(
compute_minimum_distance(T, other_geom, other_T)

Compute the minimum Euclidean distance between this geometry and another geometry.

Parameters
----------
T : numpy.ndarray, shape (4, 4)
    Homogeneous transformation of this geometry in the world (SE(3)).
other_geom : Geom
    The other geometry.
other_T : numpy.ndarray, shape (4, 4)
    Homogeneous transformation of the other geometry in the world (SE(3)).

Returns
-------
CollisionResult or None
    Signed-distance result with the two closest points **on each geometry**.
    Returns ``None`` if the pair is unsupported or filtered out.

Notes
-----
The returned distance is signed:

- distance > 0 : geometries are separated by that metric distance
- distance = 0 : geometries are just touching
- distance < 0 : geometries overlap; penetration depth = -distance

Examples
--------
>>> # quick, copy-paste friendly
>>> import numpy as np
>>> import rby1_sdk.dynamics as dyn
>>> 
>>> caps1 = dyn.GeomCapsule(height=0.4, radius=0.05, coltype=0, colaffinity=0)
>>> caps2 = dyn.GeomCapsule(height=0.4, radius=0.05, coltype=0, colaffinity=0)
>>> T1 = np.eye(4)
>>> T2 = np.eye(4); T2[0, 3] = 0.20  # shift 20 cm in x
>>> 
>>> res = caps1.compute_minimum_distance(T1, caps2, T2)
>>> if res is not None:
...     print(res.distance)
)doc")
      .def("filter", &dyn::Geom::Filter, "other_geom"_a, R"doc(
filter(other_geom)

Filter collision detection with another geometry.

Parameters
----------
other_geom : Geom
    Other geometry to filter with.

Returns
-------
bool
    True if collision should be checked, False otherwise.
)doc")
      .def("__repr__",
           [](const dyn::Geom& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             std::ostringstream out;
             out << "Geom(" << FIRST                                        //
                 << "type=" << inline_obj(py::cast(self.GetType())) << SEP  //
                 << "coltype=" << self.GetColtype() << SEP                  //
                 << "colaffinity=" << self.GetColaffinity() << LAST         //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const dyn::Geom& self) {
        using namespace rb::print;
        std::ostringstream out;
        out << "Geom(type=" << inline_obj_one_line(py::cast(self.GetType())) << ", coltype=" << self.GetColtype()
            << ", colaffinity=" << self.GetColaffinity() << ")";
        return out.str();
      });

  py::class_<dyn::GeomCapsule, dyn::Geom, std::shared_ptr<dyn::GeomCapsule>>(m, "GeomCapsule", R"doc(
Capsule geometry.

Represents a capsule (cylinder with rounded ends) for collision detection.

Attributes
----------
start_point : numpy.ndarray
    Start point of the capsule axis in meters.
end_point : numpy.ndarray
    End point of the capsule axis in meters.
radius : float
    Radius of the capsule in meters.
)doc")
      .def(py::init<double, double, unsigned int, unsigned int>(), "height"_a, "radius"_a, "coltype"_a, "colaffinity"_a,
           R"doc(
Construct a ``GeomCapsule`` with height and radius.

Parameters
----------
height : float
    Height of the capsule in meters.
radius : float
    Radius of the capsule in meters.
coltype : int
    Collision type identifier.
colaffinity : int
    Collision affinity identifier.
)doc")
      .def(py::init<Eigen::Vector3d, Eigen::Vector3d, double, unsigned int, unsigned int>(), "start_point"_a,
           "end_point"_a, "radius"_a, "coltype"_a, "colaffinity"_a, R"doc(
Construct a ``GeomCapsule`` with start and end points.

Parameters
----------
start_point : numpy.ndarray
    Start point of the capsule axis in meters.
end_point : numpy.ndarray
    End point of the capsule axis in meters.
radius : float
    Radius of the capsule in meters.
coltype : int
    Collision type identifier.
colaffinity : int
    Collision affinity identifier.
)doc")
      .def("get_start_point", &dyn::GeomCapsule::GetStartPoint, R"doc(
get_start_point()

Get the start point of the capsule axis.

Returns
-------
numpy.ndarray
    Start point in meters.
)doc")
      .def("get_end_point", &dyn::GeomCapsule::GetEndPoint, R"doc(
get_end_point()

Get the end point of the capsule axis.

Returns
-------
numpy.ndarray
    End point in meters.
)doc")
      .def("get_radius", &dyn::GeomCapsule::GetRadius, R"doc(
get_radius()

Get the radius of the capsule.

Returns
-------
float
    Radius in meters.
)doc")
      .def("__repr__",
           [](const dyn::GeomCapsule& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             std::ostringstream out;
             out << "GeomCapsule(" << FIRST
                 << "start_point=" << np_array_to_string(py::cast(self.GetStartPoint()), Style::Repr) << SEP
                 << "end_point=" << np_array_to_string(py::cast(self.GetEndPoint()), Style::Repr) << SEP
                 << "radius=" << format_number(self.GetRadius(), Style::Repr) << LAST << ")";
             return out.str();
           })
      .def("__str__", [](const dyn::GeomCapsule& self) {
        using namespace rb::print;
        std::ostringstream out;
        out << "GeomCapsule(r=" << format_number(self.GetRadius(), Style::Str) << ", "
            << "p0=" << np_array_to_string(py::cast(self.GetStartPoint()), Style::Str) << ", "
            << "p1=" << np_array_to_string(py::cast(self.GetEndPoint()), Style::Str) << ")";
        return out.str();
      });
}

void bind_collision(py::module_& m) {
  py::class_<dyn::Collision, std::shared_ptr<dyn::Collision>>(m, "Collision", R"doc(
Collision detection object.

Manages collision detection for a link with multiple geometric objects.

Attributes
----------
origin : numpy.ndarray
    Origin transformation matrix.
geoms : list
    List of geometric objects for collision detection.
)doc")
      .def(py::init<std::string>(), "name"_a, R"doc(
Construct a ``Collision`` instance.

Parameters
----------
name : str
    Name of the collision object.
)doc")
      .def("set_origin", &dyn::Collision::SetOrigin, "T"_a, R"doc(
Set the origin transformation.

Parameters
----------
T : numpy.ndarray
    Origin transformation matrix.
)doc")
      .def("get_origin", &dyn::Collision::GetOrigin, R"doc(
Get the origin transformation.

Returns
-------
numpy.ndarray
    Origin transformation matrix.
)doc")
      .def("add_geom", &dyn::Collision::AddGeom, "geom"_a, R"doc(
Add a geometric object for collision detection.

Parameters
----------
geom : Geom
    Geometric object to add.
)doc")
      .def("get_geoms",
           static_cast<std::vector<std::shared_ptr<dyn::Geom>> (dyn::Collision::*)()>(&dyn::Collision::GetGeoms), R"doc(
Get the list of geometric objects.

Returns
-------
list
    List of geometric objects.
)doc")
      .def("get_geoms",
           static_cast<const std::vector<std::shared_ptr<dyn::Geom>>& (dyn::Collision::*)() const>(
               &dyn::Collision::GetGeoms),
           R"doc(
Get the list of geometric objects (const version).

Returns
-------
list
    List of geometric objects.
)doc")
      .def("__repr__",
           [](const dyn::Collision& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             const auto geoms = self.GetGeoms();
             std::ostringstream out;
             out << "Collision(" << FIRST                                                            //
                 << "origin=" << np_array_to_string(py::cast(self.GetOrigin()), Style::Repr) << SEP  //
                 << "geoms=" << geoms.size() << LAST                                                 //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const dyn::Collision& self) {
        using namespace rb::print;
        std::ostringstream out;
        out << "Collision(geoms=" << self.GetGeoms().size() << ")";
        return out.str();
      });
}

void bind_link_joint(py::module_& m) {
  py::class_<dyn::Joint, std::shared_ptr<dyn::Joint>> joint(m, "Joint", R"doc(
Joint in the robot dynamics model.

Represents a joint connecting two links in the robot.

Attributes
----------
name : str
    Name of the joint.
parent_link : Link
    Parent link of the joint.
child_link : Link
    Child link of the joint.
)doc");

  py::class_<dyn::Link, std::shared_ptr<dyn::Link>>(m, "Link", R"doc(
Link in the robot dynamics model.

Represents a rigid body link in the robot.

Attributes
----------
name : str
    Name of the link.
inertial : Inertial
    Inertial properties of the link.
parent_joint : Joint
    Parent joint of the link.
)doc")
      .def(py::init(&dyn::Link::Make), "name"_a, "I"_a = dyn::Inertial::I(1.), R"doc(
Construct a ``Link`` instance.

Parameters
----------
name : str
    Name of the link.
I : Inertial, optional
    Inertial properties. Default is identity.
)doc")
      .def("get_name", &dyn::Link::GetName, R"doc(
get_name()

Get the name of the link.

Returns
-------
str
    Name of the link.
)doc")
      .def(
          "get_parent_joint",
          [](dyn::Link& self) -> std::shared_ptr<dyn::Joint> {
            auto ptr = self.GetParentJoint();
            if (ptr.expired()) {
              return nullptr;
            }
            return ptr.lock();
          },
          R"doc(
get_parent_joint()

Get the parent joint of the link.

Returns
-------
Joint, optional
    Parent joint if it exists, None otherwise.
)doc")
      .def("get_child_joint_list",
           static_cast<std::vector<std::shared_ptr<dyn::Joint>> (dyn::Link::*)()>(&dyn::Link::GetChildJointList), R"doc(
get_child_joint_list()

Get the list of child joints.

Returns
-------
list
    List of child joints.
)doc")
      .def("get_child_joint_list",
           static_cast<const std::vector<std::shared_ptr<dyn::Joint>>& (dyn::Link::*)() const>(
               &dyn::Link::GetChildJointList),
           py::return_value_policy::reference_internal, R"doc(
get_child_joint_list()

Get the list of child joints (const version).

Returns
-------
list
    List of child joints.
)doc")
      .def("add_collision", &dyn::Link::AddCollision, "collision"_a, R"doc(
add_collision(collision)

Add a collision object to the link.

Parameters
----------
collision : Collision
    Collision object to add.
)doc")
      .def("get_collisions",
           static_cast<std::vector<std::shared_ptr<dyn::Collision>> (dyn::Link::*)()>(&dyn::Link::GetCollisions), R"doc(
get_collisions()

Get the list of collisions.

Returns
-------
list
    List of collisions.
)doc")
      .def("get_collisions",
           static_cast<const std::vector<std::shared_ptr<dyn::Collision>>& (dyn::Link::*)() const>(
               &dyn::Link::GetCollisions),
           py::return_value_policy::reference_internal, R"doc(
get_collisions()

Get the list of collisions (const version).

Returns
-------
list
    List of collisions.
)doc")
      .def("__repr__",
           [](const dyn::Link& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             std::string parent_joint = "None";
             if (auto w = self.GetParentJoint(); !w.expired()) {
               if (auto pj = w.lock()) {
                 parent_joint = pj->GetName();
               }
             }
             const auto child_list = self.GetChildJointList();
             std::vector<std::string> child_names;
             child_names.reserve(child_list.size());
             for (const auto& w : child_list) {
               child_names.emplace_back(w->GetName());
             }

             const auto col_list = self.GetCollisions();
             std::vector<std::string> collision_names;
             collision_names.reserve(col_list.size());
             for (const auto& g : col_list) {
               if (!g) {
                 continue;
               }
               collision_names.emplace_back(g->GetName());
             }

             std::ostringstream out;
             out << "Link(" << FIRST                                                //
                 << "name=" << inline_obj(py::cast(self.GetName())) << SEP          //
                 << "parent_joint=" << inline_obj(py::cast(parent_joint)) << SEP    //
                 << "children=" << inline_obj(py::cast(child_names)) << SEP         //
                 << "collisions=" << inline_obj(py::cast(collision_names)) << LAST  //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const dyn::Link& self) {
        std::ostringstream out;
        out << "Link(" << self.GetName() << ", children=" << self.GetChildJointList().size()
            << ", collisions=" << self.GetCollisions().size() << ")";
        return out.str();
      });

  joint
      .def_static("make", &dyn::Joint::Make, "name"_a, "S"_a, R"doc(
make(name, S)

Create a joint with a specific screw axis.

Parameters
----------
name : str
    Name of the joint.
S : numpy.ndarray
    6D screw axis vector.
)doc")
      .def_static("make_revolute", &dyn::Joint::MakeRevoluteJoint, "name"_a, "T"_a = math::SE3::Identity(),
                  "axis"_a = Eigen::Vector3d{0, 0, 1}, R"doc(
make_revolute(name, T=np.identity(4), axis=np.array([0, 0, 1]))

Create a revolute joint.

Parameters
----------
name : str
    Name of the joint.
T : numpy.ndarray, optional
    Transformation from the parent link's frame to the joint's frame. Defaults to identity.
axis : numpy.ndarray, optional
    Axis of rotation. Defaults to [0, 0, 1].
)doc")
      .def_static("make_prismatic", &dyn::Joint::MakePrismaticJoint, "name"_a, "T"_a = math::SE3::Identity(),
                  "axis"_a = Eigen::Vector3d{0, 0, 1}, R"doc(
make_prismatic(name, T=np.identity(4), axis=np.array([0, 0, 1]))

Create a prismatic joint.

Parameters
----------
name : str
    Name of the joint.
T : numpy.ndarray, optional
    Transformation from the parent link's frame to the joint's frame. Defaults to identity.
axis : numpy.ndarray, optional
    Axis of translation. Defaults to [0, 0, 1].
)doc")
      .def_static("make_fixed", &dyn::Joint::MakeFixedJoint, "name"_a, R"doc(
make_fixed(name)

Create a fixed joint.

Parameters
----------
name : str
    Name of the joint.
)doc")
      .def("get_name", &dyn::Joint::GetName, R"doc(
get_name()

Get the name of the joint.

Returns
-------
str
    Name of the joint.
)doc")
      .def("connect_links", &dyn::Joint::ConnectLinks, "parent_link"_a, "child_link"_a,
           "T_pj"_a = math::SE3::Identity(), "T_jc"_a = math::SE3::Identity(), R"doc(
connect_links(parent_link, child_link, T_pj=np.identity(4), T_jc=np.identity(4))

Connect two links through this joint.

Parameters
----------
parent_link : Link
    Parent link.
child_link : Link
    Child link.
T_pj : numpy.ndarray, optional
    Transformation from parent joint to joint. Default is identity.
T_jc : numpy.ndarray, optional
    Transformation from joint to child joint. Default is identity.
)doc")
      .def("disconnect", &dyn::Joint::Disconnect, R"doc(
disconnect()

Disconnect the joint from its parent and child links.
)doc")
      .def("set_limit_q", &dyn::Joint::SetLimitQ, "lower"_a, "upper"_a, R"doc(
set_limit_q(lower, upper)

Set joint position limits.

Parameters
----------
lower : numpy.ndarray
    Lower limit for joint position.
upper : numpy.ndarray
    Upper limit for joint position.
)doc")
      .def("set_limit_qdot", &dyn::Joint::SetLimitQdot, "lower"_a, "upper"_a, R"doc(
set_limit_qdot(lower, upper)

Set joint velocity limits.

Parameters
----------
lower : numpy.ndarray
    Lower limit for joint velocity.
upper : numpy.ndarray
    Upper limit for joint velocity.
)doc")
      .def("set_limit_qddot", &dyn::Joint::SetLimitQddot, "lower"_a, "upper"_a, R"doc(
set_limit_qddot(lower, upper)

Set joint acceleration limits.

Parameters
----------
lower : numpy.ndarray
    Lower limit for joint acceleration.
upper : numpy.ndarray
    Upper limit for joint acceleration.
)doc")
      .def("set_limit_torque", &dyn::Joint::SetLimitTorque, "torque"_a, R"doc(
set_limit_torque(torque)

Set joint torque limits.

Parameters
----------
torque : numpy.ndarray
    Torque limits.
)doc")
      .def("get_limit_q_lower", &dyn::Joint::GetLimitQLower, R"doc(
get_limit_q_lower()

Get lower joint position limit.

Returns
-------
numpy.ndarray
    Lower limit for joint position.
)doc")
      .def("get_limit_q_upper", &dyn::Joint::GetLimitQUpper, R"doc(
get_limit_q_upper()

Get upper joint position limit.

Returns
-------
numpy.ndarray
    Upper limit for joint position.
)doc")
      .def("get_limit_qdot_lower", &dyn::Joint::GetLimitQdotLower, R"doc(
get_limit_qdot_lower()

Get lower joint velocity limit.

Returns
-------
numpy.ndarray
    Lower limit for joint velocity.
)doc")
      .def("get_limit_qdot_upper", &dyn::Joint::GetLimitQdotUpper, R"doc(
get_limit_qdot_upper()

Get upper joint velocity limit.

Returns
-------
numpy.ndarray
    Upper limit for joint velocity.
)doc")
      .def("get_limit_qddot_lower", &dyn::Joint::GetLimitQddotLower, R"doc(
get_limit_qddot_lower()

Get lower joint acceleration limit.

Returns
-------
numpy.ndarray
    Lower limit for joint acceleration.
)doc")
      .def("get_limit_qddot_upper", &dyn::Joint::GetLimitQddotUpper, R"doc(
get_limit_qddot_upper()

Get upper joint acceleration limit.

Returns
-------
numpy.ndarray
    Upper limit for joint acceleration.
)doc")
      .def("get_limit_torque", &dyn::Joint::GetLimitTorque, R"doc(
get_limit_torque()

Get joint torque limits.

Returns
-------
numpy.ndarray
    Torque limits.
)doc")
      .def("set_limit_q_lower", &dyn::Joint::SetLimitQLower, "val"_a, R"doc(
set_limit_q_lower(val)

Set lower joint position limit.

Parameters
----------
val : numpy.ndarray
    New lower limit for joint position.
)doc")
      .def("set_limit_q_upper", &dyn::Joint::SetLimitQUpper, "val"_a, R"doc(
set_limit_q_upper(val)

Set upper joint position limit.

Parameters
----------
val : numpy.ndarray
    New upper limit for joint position.
)doc")
      .def("set_limit_qdot_lower", &dyn::Joint::SetLimitQdotLower, "val"_a, R"doc(
set_limit_qdot_lower(val)

Set lower joint velocity limit.

Parameters
----------
val : numpy.ndarray
    New lower limit for joint velocity.
)doc")
      .def("set_limit_qdot_upper", &dyn::Joint::SetLimitQdotUpper, "val"_a, R"doc(
set_limit_qdot_upper(val)

Set upper joint velocity limit.

Parameters
----------
val : numpy.ndarray
    New upper limit for joint velocity.
)doc")
      .def("set_limit_qddot_lower", &dyn::Joint::SetLimitQddotLower, "val"_a, R"doc(
set_limit_qddot_lower(val)

Set lower joint acceleration limit.

Parameters
----------
val : numpy.ndarray
    New lower limit for joint acceleration.
)doc")
      .def("set_limit_qddot_upper", &dyn::Joint::SetLimitQddotUpper, "val"_a, R"doc(
set_limit_qddot_upper(val)

Set upper joint acceleration limit.

Parameters
----------
val : numpy.ndarray
    New upper limit for joint acceleration.
)doc")
      .def(
          "get_parent_link",
          [](dyn::Joint& self) -> std::shared_ptr<dyn::Link> {
            auto ptr = self.GetParentLink();
            if (ptr.expired()) {
              return nullptr;
            }
            return ptr.lock();
          },
          R"doc(
get_parent_link()

Get the parent link of the joint.

Returns
-------
Link, optional
    Parent link if it exists, None otherwise.
)doc")
      .def("get_child_link", static_cast<std::shared_ptr<dyn::Link> (dyn::Joint::*)()>(&dyn::Joint::GetChildLink),
           R"doc(
get_child_link()

Get the child link of the joint.

Returns
-------
Link
    Child link.
)doc")
      .def("get_child_link",
           static_cast<std::shared_ptr<const dyn::Link> (dyn::Joint::*)() const>(&dyn::Joint::GetChildLink),
           py::return_value_policy::reference_internal, R"doc(
get_child_link()

Get the child link of the joint (const version).

Returns
-------
Link
    Child link.
)doc")
      .def("is_fixed", &dyn::Joint::IsFixed, R"doc(
is_fixed()

Check if the joint is fixed.

Returns
-------
bool
    True if fixed, False otherwise.
)doc")
      .def("__repr__",
           [](const dyn::Joint& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             std::string parent = "None";
             if (auto w = self.GetParentLink(); !w.expired()) {
               if (auto p = w.lock())
                 parent = p->GetName();
             }
             std::string child = "None";
             if (auto c = self.GetChildLink()) {
               child = c->GetName();
             }

             std::ostringstream out;
             out << "Joint(" << FIRST                                       //
                 << "name=" << inline_obj(py::cast(self.GetName())) << SEP  //
                 << "fixed=" << (self.IsFixed() ? "True" : "False") << SEP  //
                 << "parent=" << inline_obj(py::cast(parent)) << SEP        //
                 << "child=" << inline_obj(py::cast(child)) << LAST         //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const dyn::Joint& self) {
        std::ostringstream out;
        out << "Joint(" << self.GetName() << (self.IsFixed() ? ", fixed" : "") << ")";
        return out.str();
      });
}

void bind_mobile_base(py::module_& m) {
  py::enum_<dyn::MobileBaseType>(m, "MobileBaseType", R"doc(
Mobile base type enumeration.

Defines the types of mobile bases supported.

Members
-------
None : int
    No mobile base.
Differential : int
    Differential drive mobile base.
Mecanum : int
    Mecanum drive mobile base.
)doc")
      .value("None", dyn::MobileBaseType::kNone, R"doc(
No mobile base.
)doc")
      .value("Differential", dyn::MobileBaseType::kDifferential, R"doc(
Differential drive mobile base.
)doc")
      .value("Mecanum", dyn::MobileBaseType::kMecanum, R"doc(
Mecanum drive mobile base.
)doc");

  py::class_<dyn::MobileBase, std::shared_ptr<dyn::MobileBase>>(m, "MobileBase", R"doc(
Base class for mobile bases.

Represents the base of the robot that can move.

Attributes
----------
type : MobileBaseType
    Type of the mobile base.
T : numpy.ndarray
    Transformation matrix from the base to the world frame.
joints : list
    List of joints that make up the mobile base.
params : dict
    Parameters of the mobile base.
)doc")
      .def_readonly("type", &dyn::MobileBase::type, R"doc(
Type of the mobile base.

Type
----
MobileBaseType
    Type of the mobile base.
)doc")
      .def_readonly("T", &dyn::MobileBase::T, R"doc(
Transformation matrix from the base to the world frame.

Type
----
numpy.ndarray
    Transformation matrix.
)doc")
      .def_readonly("joints", &dyn::MobileBase::joints, R"doc(
List of joints that make up the mobile base.

Type
----
list
    List of joint objects.
)doc")
      .def_readonly("params", &dyn::MobileBase::params, R"doc(
Parameters of the mobile base.

Type
----
dict
    Dictionary of parameters.
)doc")
      .def("__repr__",
           [](const dyn::MobileBase& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             std::ostringstream out;
             out << "MobileBase(" << FIRST                                            //
                 << "type=" << inline_obj(py::cast(self.type)) << SEP                 //
                 << "T=" << np_array_to_string(py::cast(self.T), Style::Repr) << SEP  //
                 << "joints=" << self.joints.size() << SEP                            //
                 << "params=" << inline_obj(py::cast(self.params)) << LAST            //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const dyn::MobileBase& self) {
        using namespace rb::print;
        std::ostringstream out;
        out << "MobileBase(type=" << inline_obj_one_line(py::cast(self.type)) << ", joints=" << self.joints.size()
            << ")";
        return out.str();
      });

  py::class_<dyn::MobileBaseDifferential, dyn::MobileBase, std::shared_ptr<dyn::MobileBaseDifferential>>(
      m, "MobileBaseDifferential", R"doc(
Differential drive mobile base.

Represents a differential drive mobile base with two wheels.

Attributes
----------
right_wheel_idx : int
    Index of the right wheel joint.
left_wheel_idx : int
    Index of the left wheel joint.
wheel_base : float
    Distance between the two wheels in meters.
wheel_radius : float
    Radius of the wheels in meters.
)doc")
      .def_readonly("right_wheel_idx", &dyn::MobileBaseDifferential::right_wheel_idx, R"doc(
Index of the right wheel joint.

Type
----
int
    Index of the right wheel joint.
)doc")
      .def_readonly("left_wheel_idx", &dyn::MobileBaseDifferential::left_wheel_idx, R"doc(
Index of the left wheel joint.

Type
----
int
    Index of the left wheel joint.
)doc")
      .def_readonly("wheel_base", &dyn::MobileBaseDifferential::wheel_base, R"doc(
Distance between the two wheels in meters.

Type
----
float
    Distance between the two wheels.
)doc")
      .def_readonly("wheel_radius", &dyn::MobileBaseDifferential::wheel_radius, R"doc(
Radius of the wheels in meters.

Type
----
float
    Radius of the wheels.
)doc")
      .def("__repr__",
           [](const dyn::MobileBaseDifferential& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             std::ostringstream out;
             out << "MobileBaseDifferential(" << FIRST << "type=" << inline_obj(py::cast(self.type)) << SEP  //
                 << "wheel_base=" << format_number(self.wheel_base, Style::Repr) << SEP                      //
                 << "wheel_radius=" << format_number(self.wheel_radius, Style::Repr) << SEP                  //
                 << "right_wheel_idx=" << self.right_wheel_idx << SEP                                        //
                 << "left_wheel_idx=" << self.left_wheel_idx << LAST                                         //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const dyn::MobileBaseDifferential& self) {
        using namespace rb::print;
        std::ostringstream out;
        out << "MobileBaseDifferential("
            << "base=" << format_number(self.wheel_base, Style::Str) << "m, "
            << "radius=" << format_number(self.wheel_radius, Style::Str) << "m"
            << ")";
        return out.str();
      });
}

void bind_robot_configuration(py::module_& m) {
  py::class_<dyn::RobotConfiguration>(m, "RobotConfiguration", R"doc(
Robot configuration.

Defines the base link and mobile base of the robot.

Attributes
----------
name : str
    Name of the robot configuration.
base_link : Link
    Base link of the robot.
mobile_base : MobileBase
    Mobile base of the robot.
)doc")
      .def(py::init<>(), R"doc(
Construct a ``RobotConfiguration`` instance.
)doc")
      .def(py::init([](const std::string& name, const std::shared_ptr<dyn::Link>& base_link,
                       const std::shared_ptr<dyn::MobileBase>& mobile_base) {
             auto config = std::make_unique<dyn::RobotConfiguration>();
             config->name = name;
             config->base_link = base_link;
             config->mobile_base = mobile_base;
             return config;
           }),
           "name"_a, "base_link"_a = nullptr, "mobile_base"_a = nullptr,
           R"doc(
Construct a ``RobotConfiguration`` instance.

Parameters
----------
name : str
    Name of the robot configuration.
base_link : Link
    Base link of the robot.
mobile_base : MobileBase
    Mobile base of the robot.
)doc")
      .def_readwrite("name", &dyn::RobotConfiguration::name, R"doc(
Name of the robot configuration.

Type
----
str
    Name of the robot configuration.
)doc")
      .def_readwrite("base_link", &dyn::RobotConfiguration::base_link, R"doc(
Base link of the robot.

Type
----
Link
    Base link object.
)doc")
      .def_readwrite("mobile_base", &dyn::RobotConfiguration::mobile_base, R"doc(
Mobile base of the robot.

Type
----
MobileBase
    Mobile base object.
)doc")
      .def("__repr__",
           [](const dyn::RobotConfiguration& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             std::string base_name = "None";
             if (self.base_link)
               base_name = self.base_link->GetName();

             std::string mb_type = "None";
             if (self.mobile_base) {
               mb_type = inline_obj(py::cast(self.mobile_base->type));
             }

             std::ostringstream out;
             out << "RobotConfiguration(" << FIRST                          //
                 << "name=" << inline_obj(py::cast(self.name)) << SEP       //
                 << "base_link=" << inline_obj(py::cast(base_name)) << SEP  //
                 << "mobile_base=" << mb_type << LAST                       //
                 << ")";
             return out.str();
           })
      .def("__str__", [](const dyn::RobotConfiguration& self) {
        using namespace rb::print;
        std::string base_name = self.base_link ? self.base_link->GetName() : "None";
        std::string mb_type = self.mobile_base ? inline_obj_one_line(py::cast(self.mobile_base->type)) : "None";
        std::ostringstream out;
        out << "RobotConfiguration(" << self.name << ", base=" << base_name << ", mobile=" << mb_type << ")";
        return out.str();
      });
}

template <int DOF>
void bind_robot(py::module_& m) {
  std::stringstream ss;
  ss << "Robot";

  if constexpr (DOF > 0) {
    ss << "_" << DOF;
  }

  py::class_<dyn::Robot<DOF>, std::shared_ptr<dyn::Robot<DOF>>>(m, ss.str().c_str(), R"doc(
Robot dynamics model.

Represents the dynamics of a robot with a given number of degrees of freedom.

Attributes
----------
base : Link
    Base link of the robot.
link_names : list
    List of names of all links.
joint_names : list
    List of names of all joints.
)doc")
      .def(py::init<const dyn::RobotConfiguration&>(), "robot_configuration"_a, R"doc(
Construct a Robot instance.

Parameters
----------
robot_configuration : RobotConfiguration
    Configuration of the robot.
)doc")
      .def("get_base", py::overload_cast<>(&dyn::Robot<DOF>::GetBase), R"doc(
get_base()

Get the base link of the robot.

Returns
-------
Link
    Base link.
)doc")
      .def("get_link_names", &dyn::Robot<DOF>::GetLinkNames, R"doc(
get_link_names()

Get the list of names of all links.

Returns
-------
list
    List of link names.
)doc")
      .def("get_joint_names", &dyn::Robot<DOF>::GetJointNames, R"doc(
get_joint_names()

Get the list of names of all joints.

Returns
-------
list
    List of joint names.
)doc")
      .def("get_link",
           static_cast<std::shared_ptr<dyn::Link> (dyn::Robot<DOF>::*)(const std::string&) const>(
               &dyn::Robot<DOF>::GetLink),
           "name"_a, R"doc(
get_link(name)

Get a link by name.

Parameters
----------
name : str
    Name of the link.

Returns
-------
Link, optional
    Link if found, None otherwise.
)doc")
      .def("get_link",
           static_cast<std::shared_ptr<dyn::Link> (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>, int) const>(
               &dyn::Robot<DOF>::GetLink),
           "state"_a, "index"_a, R"doc(
get_link(state, index)

Get a link by state and index.

Parameters
----------
state : State
    Current state of the robot.
index : int
    Index of the link.

Returns
-------
Link, optional
    Link if found, None otherwise.
)doc")
      .def(
          "make_state",
          [](dyn::Robot<DOF>& self, const std::vector<std::string>& link_names,
             const std::vector<std::string>& joint_names) { return self.MakeState(link_names, joint_names); },
          "link_names"_a, "joint_names"_a, R"doc(
make_state(link_names, joint_names)

Create a state from link and joint names.

The state object is essential for using the robot dynamics functions.
It stores the robot's state, its state vector (e.g., indices of joints and links), 
and also serves as a cache for intermediate results in dynamics and
kinematics calculations to optimize for speed.

Parameters
----------
link_names : list[str]
    List of link names.
joint_names : list[str]
    List of joint names.

Returns
-------
State
    A new state object.

Examples
--------
>>> import rby1_sdk.dynamics as rby_dyn
>>> import numpy as np
>>>
>>> link_0 = rby_dyn.Link("link_0")
>>> link_1 = rby_dyn.Link("link_1")
>>> 
>>> joint_0 = rby_dyn.Joint.make_revolute("joint_0", np.identity(4), np.array([0, 0, 1]))
>>> joint_0.connect_links(link_0, link_1, np.identity(4), np.identity(4))
>>> 
>>> dyn_robot = rby_dyn.Robot(
...     rby_dyn.RobotConfiguration(name="sample_robot", base_link=link_0)
... )
>>> 
>>> dyn_state = dyn_robot.make_state(["link_0", "link_1"], ["joint_0"])
>>> dyn_state.set_q(np.array([np.pi / 2]))  # Angle of joint_0 is 90 degrees
>>> 
>>> dyn_robot.compute_forward_kinematics(dyn_state)
>>> # Calculate transformation from link_0 to link_1
>>> transform = dyn_robot.compute_transformation(dyn_state, 0, 1)  # 0: link_0, 1: link_1
>>> print(transform)
)doc")
      .def("get_dof", &dyn::Robot<DOF>::GetDOF, R"doc(
get_dof()

Get the number of degrees of freedom.

Returns
-------
int
    Number of degrees of freedom.
)doc")
      .def("get_number_of_joints", &dyn::Robot<DOF>::GetNumberOfJoints, R"doc(
get_number_of_joints()

Get the number of joints.

Returns
-------
int
    Number of joints.
)doc")
      .def("compute_forward_kinematics", &dyn::Robot<DOF>::ComputeForwardKinematics, "state"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
compute_forward_kinematics(state)

Computes the forward kinematics for each joint.

This method calculates the transformation matrix from the base to each joint frame
based on the current joint positions (`q`) in the state. The results are cached
within the `state` object. This function must be called before other kinematics
or dynamics calculations.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot, which includes joint positions. This object
    will be updated with the computed transformation matrices.

Examples
--------
>>> import rby1_sdk.dynamics as rby_dyn
>>> import numpy as np
>>> # Assume dyn_robot is an initialized rby_dyn.Robot instance
>>> # and dyn_state is a corresponding state object.
>>> dyn_state.set_q(np.random.rand(dyn_robot.get_dof()))
>>> dyn_robot.compute_forward_kinematics(dyn_state)
>>> # Now you can compute transformations, Jacobians, etc.
>>> transform = dyn_robot.compute_transformation(dyn_state, 0, 1)
>>> print(transform)
)doc")
      .def("compute_diff_forward_kinematics", &dyn::Robot<DOF>::ComputeDiffForwardKinematics, "state"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
compute_diff_forward_kinematics(state)

Computes the differential forward kinematics for each joint.

This method calculates the body velocity (twist) for each joint frame based on
the current joint velocities (`qdot`) in the state. The results are cached
within the `state` object.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot, which includes joint velocities. This object
    will be updated with the computed body velocities.

Notes
-----
`compute_forward_kinematics` must be called before this function.

Examples
--------
>>> # Continuing from the previous example...
>>> dyn_state.set_qdot(np.zeros(dyn_robot.get_dof()))
>>> dyn_robot.compute_diff_forward_kinematics(dyn_state)
)doc")
      .def("compute_2nd_diff_forward_kinematics", &dyn::Robot<DOF>::Compute2ndDiffForwardKinematics, "state"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
compute_2nd_diff_forward_kinematics(state)

Computes the second-order differential forward kinematics for each joint.

This method calculates the body acceleration for each joint frame based on the
current joint accelerations (`qddot`) in the state. The results are cached
within the `state` object.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot, including joint positions, velocities, and
    accelerations. This object will be updated with the computed body accelerations.

Notes
-----
`compute_forward_kinematics` and `compute_diff_forward_kinematics` must be
called before this function.

Examples
--------
>>> # Continuing from the previous example...
>>> dyn_state.set_qddot(np.zeros(dyn_robot.get_dof()))
>>> dyn_robot.compute_2nd_diff_forward_kinematics(dyn_state)
)doc")
      .def("compute_inverse_dynamics", &dyn::Robot<DOF>::ComputeInverseDynamics, "state"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
compute_inverse_dynamics(state)

Computes the inverse dynamics of the robot.

This method calculates the joint torques required to achieve the given joint
accelerations (`qddot`), considering the current joint positions (`q`) and
velocities (`qdot`). The results are stored back into the `state` object.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot, including joint positions, velocities, and
    accelerations. This object will be updated with the computed joint torques.
    Hello. This is state.

Notes
-----
`compute_forward_kinematics`, `compute_diff_forward_kinematics`, and
`compute_2nd_diff_forward_kinematics` must be called in order before this function.

Examples
--------
>>> # This example demonstrates the full sequence for inverse dynamics.
>>> import rby1_sdk as rby
>>> import rby1_sdk.dynamics as rby_dyn
>>> import numpy as np
>>> robot = rby.create_robot_a("localhost:50051")
>>> robot.connect()
>>> dyn_robot = robot.get_dynamics()
>>> dyn_state = dyn_robot.make_state(
...     dyn_robot.get_link_names(), dyn_robot.get_joint_names()
... )
>>> q = (np.random.rand(dyn_robot.get_dof()) - 0.5) * np.pi / 2
>>> dyn_state.set_q(q)
>>> dyn_state.set_qdot(np.zeros(dyn_robot.get_dof()))
>>> dyn_state.set_qddot(np.zeros(dyn_robot.get_dof()))
>>>
>>> # Perform kinematics calculations in order
>>> dyn_robot.compute_forward_kinematics(dyn_state)
>>> dyn_robot.compute_diff_forward_kinematics(dyn_state)
>>> dyn_robot.compute_2nd_diff_forward_kinematics(dyn_state)
>>>
>>> # Compute inverse dynamics
>>> dyn_robot.compute_inverse_dynamics(dyn_state)
>>>
>>> # Get the resulting torques
>>> torques = dyn_state.get_tau()
>>> with np.printoptions(precision=4, suppress=True):
...     print(f"Inverse dynamics torque (Nm): {torques}")
)doc")
      .def("compute_gravity_term", &dyn::Robot<DOF>::ComputeGravityTerm, "state"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
compute_gravity_term(state)

Computes the gravity compensation term for the robot.

This method calculates the joint torques required to counteract gravity at the
current joint positions. The gravity vector must be set in the state object
prior to calling this function.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot, including joint positions and the gravity vector.

Returns
-------
numpy.ndarray
    A vector of joint torques required to compensate for gravity.

Notes
-----
- `compute_forward_kinematics` must be called before this function.
- The gravity vector (spatial acceleration) must be set on the `state` object
  using `state.set_gravity()` or `state.set_Vdot0()`. For standard gravity along
  the negative Z-axis, the vector is `[0, 0, 0, 0, 0, -9.81]`.

Examples
--------
>>> # Continuing from a previous example where dyn_robot and dyn_state are set up.
>>> dyn_state.set_gravity(np.array([0, 0, 0, 0, 0, -9.81]))
>>> # or dyn_state.set_Vdot0(np.array([0, 0, 0, 0, 0, 9.81]))  # Note that direction is reversed
>>> dyn_robot.compute_forward_kinematics(dyn_state)
>>> gravity_torques = dyn_robot.compute_gravity_term(dyn_state)
>>> print(gravity_torques)
)doc")
      .def("compute_mass_matrix", &dyn::Robot<DOF>::ComputeMassMatrix, "state"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
compute_mass_matrix(state)

Computes the joint space mass matrix (inertia matrix) of the robot.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot, including joint positions.

Returns
-------
numpy.ndarray
    The mass matrix (a square matrix of size DOF x DOF).

Notes
-----
`compute_forward_kinematics` must be called before this function.
)doc")
      .def("compute_reflective_inertia", &dyn::Robot<DOF>::ComputeReflectiveInertia, "state"_a,
           "reference_link_index"_a, "target_link_index"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
compute_reflective_inertia(state, reference_link_index, target_link_index)

Computes the reflective inertia (task space inertia) of the target link with respect to the reference link.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
reference_link_index : int
    The index of the reference link.
target_link_index : int
    The index of the target link.

Returns
-------
numpy.ndarray
    The 6x6 reflective inertia matrix.

Notes
-----
`compute_forward_kinematics` must be called before this function.
)doc")
      .def("compute_transformation", &dyn::Robot<DOF>::ComputeTransformation, "state"_a, "reference_link_index"_a,
           "target_link_index"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
compute_transformation(state, reference_link_index, target_link_index)

Computes the transformation matrix from a reference link to a target link.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
reference_link_index : int
    The index of the reference link (the 'from' frame).
target_link_index : int
    The index of the target link (the 'to' frame).

Returns
-------
numpy.ndarray
    The 4x4 transformation matrix (SE(3)).

Notes
-----
`compute_forward_kinematics` must be called before this function.
)doc")
      .def("compute_body_velocity", &dyn::Robot<DOF>::ComputeBodyVelocity, "state"_a, "reference_link_index"_a,
           "target_link_index"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
compute_body_velocity(state, reference_link_index, target_link_index)

Computes the relative body velocity (twist) of a target link with respect to a reference link.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
reference_link_index : int
    The index of the reference link.
target_link_index : int
    The index of the target link.

Returns
-------
numpy.ndarray
    The 6x1 body velocity vector (twist).

Notes
-----
`compute_forward_kinematics` and `compute_diff_forward_kinematics` must be
called before this function.
)doc")
      .def("compute_space_jacobian", &dyn::Robot<DOF>::ComputeSpaceJacobian, "state"_a, "reference_link_index"_a,
           "target_link_index"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
compute_space_jacobian(state, reference_link_index, target_link_index)

Computes the space Jacobian for a target link relative to a reference link.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
reference_link_index : int
    The index of the reference link.
target_link_index : int
    The index of the target link.

Returns
-------
numpy.ndarray
    The 6xDOF space Jacobian matrix.

Notes
-----
`compute_forward_kinematics` must be called before this function.
)doc")
      .def("compute_body_jacobian", &dyn::Robot<DOF>::ComputeBodyJacobian, "state"_a, "reference_link_index"_a,
           "target_link_index"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
compute_body_jacobian(state, reference_link_index, target_link_index)

Computes the body Jacobian for a target link relative to a reference link.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
reference_link_index : int
    The index of the reference link.
target_link_index : int
    The index of the target link.

Returns
-------
numpy.ndarray
    The 6xDOF body Jacobian matrix.

Notes
-----
`compute_forward_kinematics` must be called before this function.
)doc")
      .def("compute_mass", &dyn::Robot<DOF>::ComputeMass, "state"_a, "target_link_index"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
compute_mass(state, target_link_index)

Computes the mass of a specific link.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
target_link_index : int
    The index of the target link.

Returns
-------
float
    The mass of the specified link.
)doc")
      .def("compute_center_of_mass",
           static_cast<Eigen::Vector3d (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>, unsigned int,
                                                            unsigned int)>(&dyn::Robot<DOF>::ComputeCenterOfMass),
           "state"_a, "ref_link"_a, "target_link"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
compute_center_of_mass(state, ref_link, target_link)

Computes the center of mass of a single target link with respect to a reference link.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
ref_link : int
    The index of the reference link frame.
target_link : int
    The index of the target link.

Returns
-------
numpy.ndarray
    The 3D position vector of the center of mass.

Notes
-----
`compute_forward_kinematics` must be called before this function.
)doc")
      .def("compute_center_of_mass",
           static_cast<Eigen::Vector3d (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>, unsigned int,
                                                            const std::vector<unsigned int>&)>(
               &dyn::Robot<DOF>::ComputeCenterOfMass),
           "state"_a, "ref_link"_a, "target_links"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
compute_center_of_mass(state, ref_link, target_links)

Computes the combined center of mass of multiple target links with respect to a reference link.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
ref_link : int
    The index of the reference link frame.
target_links : list[int]
    A list of indices of the target links.

Returns
-------
numpy.ndarray
    The 3D position vector of the combined center of mass.

Notes
-----
`compute_forward_kinematics` must be called before this function.
)doc")
      .def("compute_center_of_mass_jacobian",
           static_cast<Eigen::Matrix<double, 3, DOF> (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>,
                                                                          unsigned int, unsigned int)>(
               &dyn::Robot<DOF>::ComputeCenterOfMassJacobian),
           "state"_a, "ref_link"_a, "target_link"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
compute_center_of_mass_jacobian(state, ref_link, target_link)

Computes the Jacobian for the center of mass of a single target link.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
ref_link : int
    The index of the reference link frame.
target_link : int
    The index of the target link.

Returns
-------
numpy.ndarray
    The 3xDOF center of mass Jacobian matrix.

Notes
-----
`compute_forward_kinematics` must be called before this function.
)doc")
      .def("compute_total_inertial", &dyn::Robot<DOF>::ComputeTotalInertial, "state"_a, "ref_link"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
compute_total_inertial(state, ref_link)

Computes the total spatial inertia of the entire robot with respect to a reference link.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
ref_link : int
    The index of the reference link frame.

Returns
-------
numpy.ndarray
    The 6x6 total spatial inertia matrix.

Notes
-----
`compute_forward_kinematics` must be called before this function.
)doc")
      .def("compute_center_of_mass",
           static_cast<Eigen::Vector3d (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>, unsigned int)>(
               &dyn::Robot<DOF>::ComputeCenterOfMass),
           "state"_a, "ref_link"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
compute_center_of_mass(state, ref_link)

Computes the center of mass of the entire robot with respect to a reference link.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
ref_link : int
    The index of the reference link frame.

Returns
-------
numpy.ndarray
    The 3D position vector of the total center of mass.

Notes
-----
`compute_forward_kinematics` must be called before this function.
)doc")
      .def("compute_center_of_mass_jacobian",
           static_cast<Eigen::Matrix<double, 3, DOF> (dyn::Robot<DOF>::*)(
               std::shared_ptr<dyn::State<DOF>>, unsigned int)>(&dyn::Robot<DOF>::ComputeCenterOfMassJacobian),
           "state"_a, "ref_link"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
compute_center_of_mass_jacobian(state, ref_link)

Computes the Jacobian for the center of mass of the entire robot.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
ref_link : int
    The index of the reference link frame.

Returns
-------
numpy.ndarray
    The 3xDOF center of mass Jacobian matrix for the whole robot.

Notes
-----
`compute_forward_kinematics` must be called before this function.
)doc")
      .def("detect_collisions_or_nearest_links", &dyn::Robot<DOF>::DetectCollisionsOrNearestLinks, "state"_a,
           "collision_threshold"_a = 0, py::call_guard<py::gil_scoped_release>(), R"doc(
detect_collisions_or_nearest_links(state, collision_threshold=0)

Detects collisions or finds the nearest links in the robot model.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
collision_threshold : int, optional
    The minimum number of link pairs to return. The function first finds all
    colliding pairs. If the number of colliding pairs is less than this
    threshold, it will supplement the result with the nearest non-colliding
    link pairs until the total count reaches the threshold. The returned list
    is always sorted by distance. If set to 0, only actual collisions are
    returned. Default is 0.

Returns
-------
list[rby1_sdk.dynamics.CollisionResult]
    A list of collision results, sorted by distance.

Notes
-----
`compute_forward_kinematics` must be called before this function.
)doc")
      .def("get_limit_q_lower", &dyn::Robot<DOF>::GetLimitQLower, "state"_a, py::call_guard<py::gil_scoped_release>(),
           R"doc(
get_limit_q_lower(state)

Gets the lower position limits for all joints.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.

Returns
-------
numpy.ndarray
    A vector of lower position limits (q) for each joint.
)doc")
      .def("get_limit_q_upper", &dyn::Robot<DOF>::GetLimitQUpper, "state"_a, py::call_guard<py::gil_scoped_release>(),
           R"doc(
get_limit_q_upper(state)

Gets the upper position limits for all joints.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.

Returns
-------
numpy.ndarray
    A vector of upper position limits (q) for each joint.
)doc")
      .def("get_limit_qdot_lower", &dyn::Robot<DOF>::GetLimitQdotLower, "state"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
get_limit_qdot_lower(state)

Gets the lower velocity limits for all joints.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.

Returns
-------
numpy.ndarray
    A vector of lower velocity limits (q_dot) for each joint.
)doc")
      .def("get_limit_qdot_upper", &dyn::Robot<DOF>::GetLimitQdotUpper, "state"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
get_limit_qdot_upper(state)

Gets the upper velocity limits for all joints.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.

Returns
-------
numpy.ndarray
    A vector of upper velocity limits (q_dot) for each joint.
)doc")
      .def("get_limit_qddot_lower", &dyn::Robot<DOF>::GetLimitQddotLower, "state"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
get_limit_qddot_lower(state)

Gets the lower acceleration limits for all joints.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.

Returns
-------
numpy.ndarray
    A vector of lower acceleration limits (q_ddot) for each joint.
)doc")
      .def("get_limit_qddot_upper", &dyn::Robot<DOF>::GetLimitQddotUpper, "state"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
get_limit_qddot_upper(state)

Gets the upper acceleration limits for all joints.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.

Returns
-------
numpy.ndarray
    A vector of upper acceleration limits (q_ddot) for each joint.
)doc")
      .def("get_limit_torque", &dyn::Robot<DOF>::GetLimitTorque, "state"_a, py::call_guard<py::gil_scoped_release>(),
           R"doc(
get_limit_torque(state)

Gets the torque limits for all joints.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.

Returns
-------
numpy.ndarray
    A vector of torque limits for each joint.
)doc")
      .def("get_joint_property", &dyn::Robot<DOF>::GetJointProperty, "state"_a, "getter"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
get_joint_property(state, getter)

Gets a specific property for all joints using a provided getter function.

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot.
getter : callable
    A function that takes a joint object and returns a double value.

Returns
-------
numpy.ndarray
    A vector containing the specified property for each joint.
)doc")
      .def("compute_mobility_inverse_diff_kinematics",
           static_cast<void (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>, const Eigen::Vector2d&, double)>(
               &dyn::Robot<DOF>::ComputeMobilityInverseDiffKinematics),
           "state"_a, "linear_velocity"_a, "angular_velocity"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
compute_mobility_inverse_diff_kinematics(state, linear_velocity, angular_velocity)

Computes the inverse differential kinematics for the mobile base.

Calculates the required wheel velocities to achieve a desired linear and angular
velocity of the mobile base. Updates the `qdot` values in the state object.

Parameters
----------
state : rby1_sdk.dynamics.State
    The robot state object to be updated.
linear_velocity : numpy.ndarray
    The desired linear velocity (x, y) in m/s.
angular_velocity : float
    The desired angular velocity (yaw) in rad/s.
)doc")
      .def("compute_mobility_inverse_diff_kinematics",
           static_cast<void (dyn::Robot<DOF>::*)(std::shared_ptr<dyn::State<DOF>>, const math::se2v::MatrixType&)>(
               &dyn::Robot<DOF>::ComputeMobilityInverseDiffKinematics),
           "state"_a, "body_velocity"_a, py::call_guard<py::gil_scoped_release>(), R"doc(
compute_mobility_inverse_diff_kinematics(state, body_velocity)

Computes the inverse differential kinematics for the mobile base from a body velocity vector.

Calculates the required wheel velocities from a desired body velocity (twist).
Updates the `qdot` values in the state object.

Parameters
----------
state : rby1_sdk.dynamics.State
    The robot state object to be updated.
body_velocity : numpy.ndarray
    The desired body velocity vector [w, vx, vy].
)doc")
      .def("compute_mobility_diff_kinematics", &dyn::Robot<DOF>::ComputeMobilityDiffKinematics, "state"_a,
           py::call_guard<py::gil_scoped_release>(), R"doc(
compute_mobility_diff_kinematics(state)

Computes the forward differential kinematics for the mobile base.

Calculates the linear and angular velocity of the mobile base from the current
wheel velocities (`qdot`).

Parameters
----------
state : rby1_sdk.dynamics.State
    The current state of the robot, including wheel velocities.

Returns
-------
numpy.ndarray
    The resulting body velocity vector [w, vx, vy].
)doc")
      .def_static("count_joints", &dyn::Robot<DOF>::CountJoints, "base_link"_a, "include_fixed"_a = false,
                  py::call_guard<py::gil_scoped_release>(), R"doc(
count_joints(base_link, include_fixed=False)

Counts the number of joints in a kinematic chain starting from a base link.

Parameters
----------
base_link : rby1_sdk.dynamics.Link
    The starting link of the kinematic chain.
include_fixed : bool, optional
    Whether to include fixed joints in the count. Default is False.

Returns
-------
int
    The total number of joints.
)doc")
      .def("__repr__",
           [](const dyn::Robot<DOF>& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             auto base = self.GetBase();
             std::string base_name = base ? base->GetName() : "None";
             const auto links = self.GetLinkNames();
             const auto joints = self.GetJointNames();

             std::ostringstream out;
             out << "Robot(" << FIRST << "dof=" << self.GetDOF() << SEP  //
                 << "links=" << links.size() << SEP                      //
                 << "joints=" << joints.size() << SEP                    //
                 << "base=" << inline_obj(py::cast(base_name))           //
                 << LAST << ")";
             return out.str();
           })
      .def("__str__", [](const dyn::Robot<DOF>& self) {
        auto base = self.GetBase();
        std::string base_name = base ? base->GetName() : "None";
        std::ostringstream out;
        out << "Robot(dof=" << self.GetDOF() << ", joints=" << self.GetNumberOfJoints() << ", base=" << base_name
            << ")";
        return out.str();
      });
}

template <int DOF>
void bind_state(py::module_& m) {
  std::stringstream ss;
  ss << "State";

  if constexpr (DOF > 0) {
    ss << "_" << DOF;
  }

  py::class_<dyn::State<DOF>, std::shared_ptr<dyn::State<DOF>>>(m, ss.str().c_str(), R"doc(
Robot state for dynamics calculations.

This class stores the state of the robot, including joint positions, velocities,
accelerations, and torques. It also serves as a cache for intermediate results
in dynamics and kinematics calculations to optimize performance.
)doc")
      .def_property_readonly("base_link_idx", &dyn::State<DOF>::GetBaseLinkIdx, R"doc(
Index of the base link.

Type
----
int
)doc")
      .def_property(
          "q", [](dyn::State<DOF>& self) { return self.GetQ(); },
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> q) { self.SetQ(q); }, R"doc(
Joint positions.

Type
----
numpy.ndarray
    A vector of size DOF.
)doc")
      .def_property(
          "qdot", [](dyn::State<DOF>& self) { return self.GetQdot(); },
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> qdot) { self.SetQdot(qdot); }, R"doc(
Joint velocities.

Type
----
numpy.ndarray
    A vector of size DOF.
)doc")
      .def_property(
          "qddot", [](dyn::State<DOF>& self) { return self.GetQddot(); },
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> qddot) { self.SetQddot(qddot); }, R"doc(
Joint accelerations.

Type
----
numpy.ndarray
    A vector of size DOF.
)doc")
      .def_property(
          "tau", [](dyn::State<DOF>& self) { return self.GetTau(); },
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> tau) { self.SetTau(tau); }, R"doc(
Joint torques. This is typically an output of inverse dynamics.

Type
----
numpy.ndarray
    A vector of size DOF.
)doc")
      .def_property(
          "V0", [](dyn::State<DOF>& self) { return self.GetV0(); },
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> V0) { self.SetV0(V0); }, R"doc(
Spatial velocity of the base link (twist).

Type
----
numpy.ndarray
    A 6D vector.
)doc")
      .def_property(
          "Vdot0", [](dyn::State<DOF>& self) { return self.GetVdot0(); },
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> Vdot0) { self.SetVdot0(Vdot0); }, R"doc(
Spatial acceleration of the base link. It is used to specify external forces like gravity.
Note that `gravity = -Vdot0`.

Type
----
numpy.ndarray
    A 6D vector.
)doc")
      .def_property_readonly("joint_names", &dyn::State<DOF>::GetJointNames, R"doc(
List of joint names associated with this state.

Type
----
list[str]
)doc")
      .def_property_readonly("link_names", &dyn::State<DOF>::GetLinkNames, R"doc(
List of link names associated with this state.

Type
----
list[str]
)doc")
      .def("get_base_link_idx", &dyn::State<DOF>::GetBaseLinkIdx, R"doc(
get_base_link_idx()

Get the index of the base link.

Returns
-------
int
    Index of the base link.
)doc")
      .def(
          "set_q", [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> q) { self.SetQ(q); }, "q"_a, R"doc(
set_q(q)

Set the joint positions.

Parameters
----------
q : numpy.ndarray
    Joint positions vector.
)doc")
      .def("get_q", &dyn::State<DOF>::GetQ, R"doc(
get_q()

Get the joint positions.

Returns
-------
numpy.ndarray
    Joint positions vector.
)doc")
      .def(
          "set_qdot", [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> qdot) { self.SetQdot(qdot); },
          "qdot"_a, R"doc(
set_qdot(qdot)

Set the joint velocities.

Parameters
----------
qdot : numpy.ndarray
    Joint velocities vector.
)doc")
      .def("get_qdot", &dyn::State<DOF>::GetQdot, R"doc(
get_qdot()

Get the joint velocities.

Returns
-------
numpy.ndarray
    Joint velocities vector.
)doc")
      .def(
          "set_qddot", [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> qddot) { self.SetQddot(qddot); },
          "qddot"_a, R"doc(
set_qddot(qddot)

Set the joint accelerations.

Parameters
----------
qddot : numpy.ndarray
    Joint accelerations vector.
)doc")
      .def("get_qddot", &dyn::State<DOF>::GetQddot, R"doc(
get_qddot()

Get the joint accelerations.

Returns
-------
numpy.ndarray
    Joint accelerations vector.
)doc")
      .def(
          "set_tau", [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> tau) { self.SetTau(tau); }, "tau"_a,
          R"doc(
set_tau(tau)

Set the joint torques.

Parameters
----------
tau : numpy.ndarray
    Joint torques vector.
)doc")
      .def("get_tau", &dyn::State<DOF>::GetTau, R"doc(
get_tau()

Get the joint torques.

Returns
-------
numpy.ndarray
    Joint torques vector.
)doc")
      .def(
          "set_V0", [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::VectorXd> V0) { self.SetV0(V0); }, "V0"_a, R"doc(
set_V0(V0)

Set the spatial velocity of the base link.

Parameters
----------
V0 : numpy.ndarray
    6D spatial velocity vector (twist).
)doc")
      .def(
          "set_Vdot0",
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::Vector<double, 6>> Vdot0) { self.SetVdot0(Vdot0); },
          "Vdot0"_a, R"doc(
set_Vdot0(Vdot0)

Set the spatial acceleration of the base link.

Parameters
----------
Vdot0 : numpy.ndarray
    6D spatial acceleration vector.
)doc")
      .def(
          "set_gravity",
          [](dyn::State<DOF>& self, Eigen::Ref<const Eigen::Vector<double, 6>> gravity) { self.SetGravity(gravity); },
          "gravity"_a, R"doc(
set_gravity(gravity)

Set the gravity vector. This is a convenience function that sets `Vdot0 = -gravity`.

Parameters
----------
gravity : numpy.ndarray
    6D gravity vector (e.g., `[0, 0, 0, 0, 0, -9.81]`).
)doc")
      .def("get_joint_names", &dyn::State<DOF>::GetJointNames, R"doc(
get_joint_names()

Get the list of joint names associated with this state.

Returns
-------
list[str]
    List of joint names.
)doc")
      .def("get_link_names", &dyn::State<DOF>::GetLinkNames, R"doc(
get_link_names()

Get the list of link names associated with this state.

Returns
-------
list[str]
    List of link names.
)doc")
      .def("__repr__",
           [](const dyn::State<DOF>& self) {
             using namespace rb::print;
             const bool ml = use_multiline_repr();
             const char* FIRST = ml ? "\n  " : "";
             const char* SEP = ml ? ",\n  " : ", ";
             const char* LAST = ml ? "\n" : "";

             std::ostringstream out;
             out << "State(" << FIRST  //
                 << "base_link_idx=" << self.GetBaseLinkIdx() << SEP;

             {
               std::string k = "q=";
               std::string v = np_array_to_string(py::cast(self.GetQ()), Style::Repr);
               out << k << indent_continuation(v, (int)(ml ? 2 + k.size() : 0));
               out << SEP;
             }
             {
               std::string k = "qdot=";
               std::string v = np_array_to_string(py::cast(self.GetQdot()), Style::Repr);
               out << k << indent_continuation(v, (int)(ml ? 2 + k.size() : 0));
               out << SEP;
             }
             {
               std::string k = "qddot=";
               std::string v = np_array_to_string(py::cast(self.GetQddot()), Style::Repr);
               out << k << indent_continuation(v, (int)(ml ? 2 + k.size() : 0));
               out << SEP;
             }
             {
               std::string k = "tau=";
               std::string v = np_array_to_string(py::cast(self.GetTau()), Style::Repr);
               out << k << indent_continuation(v, (int)(ml ? 2 + k.size() : 0));
               out << SEP;
             }
             {
               std::string k = "V0=";
               std::string v = np_array_to_string(py::cast(self.GetV0()), Style::Repr);
               out << k << indent_continuation(v, (int)(ml ? 2 + k.size() : 0));
               out << SEP;
             }
             {
               std::string k = "Vdot0=";
               std::string v = np_array_to_string(py::cast(self.GetVdot0()), Style::Repr);
               out << k << indent_continuation(v, (int)(ml ? 2 + k.size() : 0));
             }

             out << LAST << ")";
             return out.str();
           })
      .def("__str__", [](const dyn::State<DOF>& self) {
        using namespace rb::print;
        std::ostringstream out;
        out << "State(base=" << self.GetBaseLinkIdx() << ", q=" << np_array_to_string(py::cast(self.GetQ()), Style::Str)
            << ", qdot=" << np_array_to_string(py::cast(self.GetQdot()), Style::Str)
            << ", tau=" << np_array_to_string(py::cast(self.GetTau()), Style::Str) << ")";
        return out.str();
      });
}

void pybind11_dynamics(py::module_& m) {
  bind_collision_result(m);
  bind_geom(m);
  bind_collision(m);

  bind_link_joint(m);
  bind_mobile_base(m);
  bind_robot_configuration(m);

  bind_state<-1>(m);
  bind_robot<-1>(m);

  bind_state<y1_model::A::kRobotDOF>(m);
  bind_robot<y1_model::A::kRobotDOF>(m);

  bind_state<y1_model::T5::kRobotDOF>(m);
  bind_robot<y1_model::T5::kRobotDOF>(m);

  bind_state<y1_model::M::kRobotDOF>(m);
  bind_robot<y1_model::M::kRobotDOF>(m);

  bind_state<y1_model::UB::kRobotDOF>(m);
  bind_robot<y1_model::UB::kRobotDOF>(m);

  m.def("load_robot_from_urdf_data", &dyn::LoadRobotFromURDFData, "model"_a, "base_link_name"_a,
        R"doc(
load_robot_from_urdf_data(model, base_link_name)

Load a robot model from URDF data string.

This function parses URDF XML content directly from a string and
constructs a robot model. It is useful when URDF data is already
loaded in memory and does not need to be read from a file.

Parameters
----------
model : str
    URDF XML data as a string.
base_link_name : str
    Name of the base link in the URDF. This will be used as the
    reference link of the robot.

Returns
-------
Robot
    The loaded robot model instance.

Examples
--------
>>> with open("robot.urdf") as f:
...     urdf_data = f.read()
>>> robot = load_robot_from_urdf_data(urdf_data, "base_link")
)doc");

  m.def("load_robot_from_urdf", &dyn::LoadRobotFromURDF, "path"_a, "base_link_name"_a,
        R"doc(
load_robot_from_urdf(path, base_link_name)

Load a robot model from a URDF file.

This function reads a URDF file from the given path and constructs
a robot model based on its description.

Parameters
----------
path : str
    File system path to the URDF file.
base_link_name : str
    Name of the base link in the URDF. This will be used as the
    reference link of the robot.

Returns
-------
Robot
    The loaded robot model instance.

Examples
--------
>>> robot = load_robot_from_urdf("path/to/robot.urdf", "base_link")
)doc");
}