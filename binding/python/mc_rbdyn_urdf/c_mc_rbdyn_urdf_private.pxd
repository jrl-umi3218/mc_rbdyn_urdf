#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from c_mc_rbdyn_urdf cimport *

cdef extern from "mc_rbdyn_urdf_wrapper.hpp" namespace "mc_rbdyn_urdf":
  const GeometryMesh& getMesh(const Geometry&)
  const GeometryBox& getBox(const Geometry&)
  const GeometryCylinder& getCylinder(const Geometry&)
  const GeometrySphere& getSphere(const Geometry&)
  void setMesh(Geometry&, const GeometryMesh&)
  void setBox(Geometry&, const GeometryBox&)
  void setCylinder(Geometry&, const GeometryCylinder&)
  void setSphere(Geometry&, const GeometrySphere&)
