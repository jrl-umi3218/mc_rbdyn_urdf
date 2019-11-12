/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE MC_RBDYN_URDF

#include <mc_rbdyn_urdf/urdf.h>

#include <boost/test/unit_test.hpp>

// XYZSarm Robot

//                b4
//             j3 | RevX
//  Root     j0   |   j1     j2
//  ---- b0 ---- b1 ---- b2 ----b3
//  Fixed    RevX   RevY    RevZ

//  X
//  ^
//  |
//   -- > Y

std::string XYZSarmUrdf(
    R"(<robot name="XYZSarm">
    <link name="b0">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0 0" />
        <mass value="1" />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
      <visual>
        <origin rpy="0. 0. 0." xyz=".1 .2 .3"/>
        <geometry>
          <mesh filename="test_mesh1.dae"/>
        </geometry>
      </visual>
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
        <geometry>
          <mesh filename="/some/longlonglong/longlonglong/longlonglong/path/test_mesh2.dae"/>
        </geometry>
      </visual>
      <visual>
        <origin rpy="0 0 0" xyz="0 0 0"/>
      </visual>
    </link>
    <link name="b1">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0.5 0" />
        <mass value="5." />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>
    <link name="b2">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0.5 0" />
        <mass value="2." />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>
    <link name="b3">
      <inertial>
        <origin rpy="0 0 0" xyz="0 0.5 0" />
        <mass value="1.5" />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>
    <link name="b4">
      <inertial>
        <origin rpy="0 0 0" xyz="0.5 0 0" />
        <mass value="1" />
        <inertia ixx="0.1" ixy="0.0" ixz="0.0"
                iyy="0.05" iyz="0.0" izz="0.001" />
      </inertial>
    </link>
    <joint name="j0" type="revolute">
      <parent link="b0" />
      <child link="b1" />
      <origin rpy="0 0 0" xyz="0 1 0" />
      <axis xyz="1 0 0" />
      <limit lower="-1" upper="1" velocity="10" effort="50" />
    </joint>
    <joint name="j1" type="revolute">
      <parent link="b1" />
      <child link="b2" />
      <origin rpy="0 0 0" xyz="0 1 0" />
      <axis xyz="0 1 0" />
      <limit lower="-1" upper="1" velocity="10" effort="50" />
    </joint>
    <joint name="j2" type="revolute">
      <parent link="b2" />
      <child link="b3" />
      <origin rpy="0 0 0" xyz="0 1 0" />
      <axis xyz="0 0 1" />
      <limit lower="-1" upper="1" velocity="10" effort="50" />
    </joint>
    <joint name="j3" type="continuous">
      <parent link="b1" />
      <child link="b4" />
      <origin rpy="1. 0 0" xyz="1 0 0" />
      <axis xyz="1 0 0" />
    </joint>
  </robot>
)");

namespace mc_rbdyn_urdf
{
bool operator==(const Geometry::Mesh & m1, const Geometry::Mesh & m2)
{
  return m1.scale == m2.scale && m1.filename == m2.filename;
}

bool operator==(const Geometry::Box & b1, const Geometry::Box & b2)
{
  return b1.size == b2.size;
}

bool operator==(const Geometry::Sphere & b1, const Geometry::Sphere & b2)
{
  return b1.radius == b2.radius;
}

bool operator==(const Geometry::Cylinder & b1, const Geometry::Cylinder & b2)
{
  return b1.radius == b2.radius && b1.length == b2.length;
}

bool operator==(const Geometry& g1, const Geometry& g2)
{
  if(g1.type != g2.type) return false;
  switch(g1.type)
  {
    case Geometry::MESH:
      return g1.data.m == g2.data.m;
    case Geometry::BOX:
      return g1.data.b == g2.data.b;
    case Geometry::SPHERE:
      return g1.data.s == g2.data.s;
    case Geometry::CYLINDER:
      return g1.data.c == g2.data.c;
    default:
      return true;
  };
}

bool operator==(const Visual & v1, const Visual & v2)
{
  return v1.name == v2.name && v1.origin == v2.origin && v1.geometry == v2.geometry;
}

} // namespace mc_rbdyn_urdf

mc_rbdyn_urdf::URDFParserResult createRobot()
{
  mc_rbdyn_urdf::URDFParserResult res;

  Eigen::Matrix3d I0, I1, I2, I3, I4;

  I0 << 0.1, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.001;

  I1 << 1.35, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 1.251;

  I2 << 0.6, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.501;

  I3 << 0.475, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.376;

  I4 << 0.1, 0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.251;
  Eigen::Affine3d T0, T1;
  T0.matrix() << 1, 0, 0, .1, 0, 1, 0, .2, 0, 0, 1, .3, 0, 0, 0, 1;
  T1 = Eigen::Affine3d::Identity();

  rbd::Body b0(1., Eigen::Vector3d::Zero(), I0, "b0");
  rbd::Body b1(5., Eigen::Vector3d(0., 0.5, 0.), I1, "b1");
  rbd::Body b2(2., Eigen::Vector3d(0., 0.5, 0.), I2, "b2");
  rbd::Body b3(1.5, Eigen::Vector3d(0., 0.5, 0.), I3, "b3");
  rbd::Body b4(1., Eigen::Vector3d(0.5, 0., 0.), I4, "b4");

  res.mbg.addBody(b0);
  res.mbg.addBody(b1);
  res.mbg.addBody(b2);
  res.mbg.addBody(b3);
  res.mbg.addBody(b4);

  rbd::Joint j0(rbd::Joint::RevX, true, "j0");
  rbd::Joint j1(rbd::Joint::RevY, true, "j1");
  rbd::Joint j2(rbd::Joint::RevZ, true, "j2");
  rbd::Joint j3(rbd::Joint::RevX, true, "j3");

  res.mbg.addJoint(j0);
  res.mbg.addJoint(j1);
  res.mbg.addJoint(j2);
  res.mbg.addJoint(j3);

  sva::PTransformd to(Eigen::Vector3d(0., 1., 0.));
  sva::PTransformd from(sva::PTransformd::Identity());

  res.mbg.linkBodies("b0", to, "b1", from, "j0");
  res.mbg.linkBodies("b1", to, "b2", from, "j1");
  res.mbg.linkBodies("b2", to, "b3", from, "j2");
  res.mbg.linkBodies("b1", sva::PTransformd(sva::RotX(1.), Eigen::Vector3d(1., 0., 0.)), "b4", from, "j3");

  res.limits.lower = {{"j0", {-1.}}, {"j1", {-1.}}, {"j2", {-1.}}};
  res.limits.upper = {{"j0", {1.}}, {"j1", {1.}}, {"j2", {1.}}};
  res.limits.velocity = {{"j0", {10.}}, {"j1", {10.}}, {"j2", {10.}}};
  res.limits.torque = {{"j0", {50.}}, {"j1", {50.}}, {"j2", {50.}}};

  mc_rbdyn_urdf::Visual v1, v2;
  v1.origin = sva::PTransformd(T0.rotation(), T0.translation());
  v1.geometry.type = mc_rbdyn_urdf::Geometry::Type::MESH;
  new (&v1.geometry.data.m) mc_rbdyn_urdf::Geometry::Mesh{"test_mesh1.dae", 1.0};

  v2.origin = sva::PTransformd(T1.rotation(), T1.translation());
  v2.geometry.type = mc_rbdyn_urdf::Geometry::Type::MESH;
  new (&v2.geometry.data.m) mc_rbdyn_urdf::Geometry::Mesh{"/some/longlonglong/longlonglong/longlonglong/path/test_mesh2.dae", 1.0};

  res.visual = {{"b0", {v1, v2}}};

  res.mb = res.mbg.makeMultiBody("b0", true);
  res.mbc = rbd::MultiBodyConfig(res.mb);
  res.mbc.zero(res.mb);

  return res;
}

const double TOL = 1e-6;

BOOST_AUTO_TEST_CASE(loadTest)
{
  auto cppRobot = createRobot();
  auto strRobot = mc_rbdyn_urdf::rbdyn_from_urdf(XYZSarmUrdf);

  BOOST_CHECK_EQUAL(cppRobot.mb.nrBodies(), strRobot.mb.nrBodies());
  BOOST_CHECK_EQUAL(cppRobot.mb.nrJoints(), strRobot.mb.nrJoints());
  BOOST_CHECK_EQUAL(cppRobot.mb.nrParams(), strRobot.mb.nrParams());
  BOOST_CHECK_EQUAL(cppRobot.mb.nrDof(), strRobot.mb.nrDof());

  BOOST_CHECK(std::equal(cppRobot.mb.predecessors().begin(), cppRobot.mb.predecessors().end(),
                         strRobot.mb.predecessors().begin()));
  BOOST_CHECK(
      std::equal(cppRobot.mb.successors().begin(), cppRobot.mb.successors().end(), strRobot.mb.successors().begin()));
  BOOST_CHECK(std::equal(cppRobot.mb.parents().begin(), cppRobot.mb.parents().end(), strRobot.mb.parents().begin()));
  BOOST_CHECK(
      std::equal(cppRobot.mb.transforms().begin(), cppRobot.mb.transforms().end(), strRobot.mb.transforms().begin()));

  BOOST_CHECK(std::equal(cppRobot.limits.lower.begin(), cppRobot.limits.lower.end(), strRobot.limits.lower.begin()));
  BOOST_CHECK(std::equal(cppRobot.limits.upper.begin(), cppRobot.limits.upper.end(), strRobot.limits.upper.begin()));
  BOOST_CHECK(
      std::equal(cppRobot.limits.velocity.begin(), cppRobot.limits.velocity.end(), strRobot.limits.velocity.begin()));
  BOOST_CHECK(std::equal(cppRobot.limits.torque.begin(), cppRobot.limits.torque.end(), strRobot.limits.torque.begin()));

  for(int i = 0; i < cppRobot.mb.nrBodies(); ++i)
  {
    const auto & b1 = cppRobot.mb.body(i);
    const auto & b2 = strRobot.mb.body(i);

    BOOST_CHECK_EQUAL(b1.name(), b2.name());

    BOOST_CHECK_EQUAL(b1.inertia().mass(), b2.inertia().mass());
    BOOST_CHECK_EQUAL(b1.inertia().momentum(), b2.inertia().momentum());
    BOOST_CHECK_SMALL((b1.inertia().inertia() - b2.inertia().inertia()).norm(), TOL);
  }

  for(int i = 0; i < cppRobot.mb.nrJoints(); ++i)
  {
    const auto & j1 = cppRobot.mb.joint(i);
    const auto & j2 = strRobot.mb.joint(i);

    BOOST_CHECK_EQUAL(j1.name(), j2.name());
    BOOST_CHECK_EQUAL(j1.type(), j2.type());
    BOOST_CHECK_EQUAL(j1.direction(), j2.direction());
    BOOST_CHECK_EQUAL(j1.motionSubspace(), j2.motionSubspace());
  }
}

BOOST_AUTO_TEST_CASE(visualTest)
{
  auto cppRobot = createRobot();
  auto strRobot = mc_rbdyn_urdf::rbdyn_from_urdf(XYZSarmUrdf);
  const auto & cpp_geometries = cppRobot.visual;
  const auto & str_geometries = strRobot.visual;

  BOOST_CHECK_EQUAL(str_geometries.size(), cpp_geometries.size());
  for(const auto & g : str_geometries)
  {
    BOOST_CHECK_EQUAL(g.second.size(), cpp_geometries.at(g.first).size());
  }

  for(const auto & body : cppRobot.mb.bodies())
  {
    BOOST_CHECK(std::equal(strRobot.visual[body.name()].begin(), strRobot.visual[body.name()].end(),
                           cppRobot.visual[body.name()].begin()));
  }
}
