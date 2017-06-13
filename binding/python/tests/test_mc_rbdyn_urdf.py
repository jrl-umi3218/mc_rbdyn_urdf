#!/usr/bin/env python

# Copyright 2015-2017 CNRS-UM LIRMM, CNRS-AIST JRL
#
# This file is part of mc_rbdyn_urdf.
#
# mc_rbdyn_urdf is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# mc_rbdyn_urdf is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with mc_rbdyn_urdf.  If not, see <http://www.gnu.org/licenses/>.

import unittest

import eigen
import sva
import rbdyn

import mc_rbdyn_urdf
mc_rbdyn_urdf.EXPERT_MODE(True)

XYZSarmUrdf = '''
<robot name="XYZSarm">
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
          <mesh filename="test_mesh2.dae"/>
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
'''

def createRobot():
  mb, mbc, mbg = rbdyn.MultiBody(), rbdyn.MultiBodyConfig(), rbdyn.MultiBodyGraph()
  limits = mc_rbdyn_urdf.Limits()
  visual = {}
  collision_tf = {}

  I0 = eigen.Matrix3d([[0.1, 0.0, 0.0], [0.0, 0.05, 0.0], [0.0, 0.0, 0.001]])
  I1 = eigen.Matrix3d([[1.35, 0.0, 0.0], [0.0, 0.05, 0.0], [0.0, 0.0, 1.251]])
  I2 = eigen.Matrix3d([[0.6, 0.0, 0.0], [0.0, 0.05, 0.0], [0.0, 0.0, 0.501]])
  I3 = eigen.Matrix3d([[0.475, 0.0, 0.0], [0.0, 0.05, 0.0], [0.0, 0.0, 0.376]])
  I4 = eigen.Matrix3d([[0.1, 0.0, 0.0], [0.0, 0.3, 0.0], [0.0, 0.0, 0.251]])

  T0 = sva.PTransformd(eigen.Vector3d(0.1, 0.2, 0.3))
  T1 = sva.PTransformd.Identity()

  b0 = rbdyn.Body(1., eigen.Vector3d.Zero(), I0, "b0")
  b1 = rbdyn.Body(5., eigen.Vector3d(0., 0.5, 0.), I1, "b1")
  b2 = rbdyn.Body(2., eigen.Vector3d(0., 0.5, 0.), I2, "b2")
  b3 = rbdyn.Body(1.5, eigen.Vector3d(0., 0.5, 0.), I3, "b3")
  b4 = rbdyn.Body(1., eigen.Vector3d(0.5, 0., 0.), I4, "b4")

  mbg.addBody(b0)
  mbg.addBody(b1)
  mbg.addBody(b2)
  mbg.addBody(b3)
  mbg.addBody(b4)

  j0 = rbdyn.Joint(rbdyn.Joint.Rev, eigen.Vector3d.UnitX(), True, "j0")
  j1 = rbdyn.Joint(rbdyn.Joint.Rev, eigen.Vector3d.UnitY(), True, "j1")
  j2 = rbdyn.Joint(rbdyn.Joint.Rev, eigen.Vector3d.UnitZ(), True, "j2")
  j3 = rbdyn.Joint(rbdyn.Joint.Rev, eigen.Vector3d.UnitX(), True, "j3")

  mbg.addJoint(j0)
  mbg.addJoint(j1)
  mbg.addJoint(j2)
  mbg.addJoint(j3)

  to = sva.PTransformd(eigen.Vector3d(0, 1, 0))
  from_ = sva.PTransformd.Identity()

  mbg.linkBodies("b0", to, "b1", from_, "j0")
  mbg.linkBodies("b1", to, "b2", from_, "j1")
  mbg.linkBodies("b2", to, "b3", from_, "j2")
  mbg.linkBodies("b1", sva.PTransformd(sva.RotX(1.), eigen.Vector3d(1.,0.,0.)), "b4", from_, "j3")

  mb = mbg.makeMultiBody("b0", True)
  mbc = rbdyn.MultiBodyConfig(mb)
  mbc.zero(mb)

  limits.lower = {"j0": [-1.], "j1": [-1.], "j2": [-1.], "j3": [-float('Inf')]}
  limits.upper = {"j0": [1.], "j1": [1.], "j2": [1.], "j3": [float('Inf')]}
  limits.velocity = {"j0": [10.], "j1": [10.], "j2": [10.], "j3": [float('Inf')]}
  limits.torque = {"j0": [50.], "j1": [50.], "j2": [50.], "j3": [float('Inf')]}

  v1 = mc_rbdyn_urdf.Visual()
  v1.origin = T0
  geometry = mc_rbdyn_urdf.Geometry()
  geometry.type = mc_rbdyn_urdf.Geometry.MESH
  mesh = mc_rbdyn_urdf.GeometryMesh()
  mesh.filename = "test_mesh1.dae"
  geometry.data = mesh
  v1.geometry = geometry

  v2 = mc_rbdyn_urdf.Visual()
  v2.origin = T1
  mesh.filename = "test_mesh2.dae"
  geometry.data = mesh
  v2.geometry = geometry

  visual = {b"b0": [v1, v2]}

  return mb, mbc, mbg, limits, visual, collision_tf

TOL = 1e-6

class LoadTest(unittest.TestCase):
  def test(self):
    mb, mbc, mbg, limits, visual, collision_tf = createRobot()
    res = mc_rbdyn_urdf.rbdyn_from_urdf(XYZSarmUrdf, baseLink = "b0")

    self.assertEqual(mb.nrBodies(), res.mb.nrBodies())
    self.assertEqual(mb.nrJoints(), res.mb.nrJoints())
    self.assertEqual(mb.nrParams(), res.mb.nrParams())
    self.assertEqual(mb.nrDof(), res.mb.nrDof())

    self.assertEqual(mb.predecessors(), res.mb.predecessors())
    self.assertEqual(mb.successors(), res.mb.successors())
    self.assertEqual(mb.parents(), res.mb.parents())
    self.assertEqual(mb.transforms(), res.mb.transforms())

    self.assertEqual(limits.lower, res.limits.lower)
    self.assertEqual(limits.upper, res.limits.upper)
    self.assertEqual(limits.velocity, res.limits.velocity)
    self.assertEqual(limits.torque, res.limits.torque)

    for i in range(mb.nrBodies()):
      b1 = mb.body(i)
      b2 = res.mb.body(i)

      self.assertEqual(b1.name(), b2.name())
      self.assertEqual(b1.inertia().mass(), b2.inertia().mass())
      self.assertEqual(b1.inertia().momentum(), b2.inertia().momentum())
      self.assertEqual(b1.inertia().inertia(), b2.inertia().inertia())

    for i in range(mb.nrJoints()):
      j1 = mb.joint(i)
      j2 = res.mb.joint(i)

      self.assertEqual(j1.name(), j2.name())
      self.assertEqual(j1.type(), j2.type())
      self.assertEqual(j1.direction(), j2.direction())
      self.assertEqual(j1.motionSubspace(), j2.motionSubspace())

class VisualTest(unittest.TestCase):
  def test(self):
    mb, mbc, mbg, limits, visual, collision_tf = createRobot()
    res = mc_rbdyn_urdf.rbdyn_from_urdf(XYZSarmUrdf, baseLink = "b0")

    self.assertEqual(visual, res.visual)

if __name__ == "__main__":
  suite = unittest.TestSuite()
  suite.addTest(LoadTest('test'))
  suite.addTest(VisualTest('test'))
  unittest.TextTestRunner(verbosity=2).run(suite)
