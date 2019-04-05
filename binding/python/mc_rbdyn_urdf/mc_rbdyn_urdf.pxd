#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport c_mc_rbdyn_urdf

cdef class Limits(object):
  cdef c_mc_rbdyn_urdf.Limits impl

cdef Limits LimitsFromC(const c_mc_rbdyn_urdf.Limits&)

cdef class GeometryMesh(object):
  cdef c_mc_rbdyn_urdf.GeometryMesh impl

cdef GeometryMesh GeometryMeshFromC(const c_mc_rbdyn_urdf.GeometryMesh&)

cdef class GeometryBox(object):
  cdef c_mc_rbdyn_urdf.GeometryBox impl

cdef GeometryBox GeometryBoxFromC(const c_mc_rbdyn_urdf.GeometryBox&)

cdef class GeometryCylinder(object):
  cdef c_mc_rbdyn_urdf.GeometryCylinder impl

cdef GeometryCylinder GeometryCylinderFromC(const c_mc_rbdyn_urdf.GeometryCylinder&)

cdef class GeometrySphere(object):
  cdef c_mc_rbdyn_urdf.GeometrySphere impl

cdef GeometrySphere GeometrySphereFromC(const c_mc_rbdyn_urdf.GeometrySphere&)

cdef class Geometry(object):
  cdef c_mc_rbdyn_urdf.Geometry impl

cdef Geometry GeometryFromC(const c_mc_rbdyn_urdf.Geometry&)

cdef class Visual(object):
  cdef c_mc_rbdyn_urdf.Visual impl

cdef Visual VisualFromC(const c_mc_rbdyn_urdf.Visual&)

cdef class URDFParserResult(object):
  cdef c_mc_rbdyn_urdf.URDFParserResult impl

cdef URDFParserResult URDFParserResultFromC(const c_mc_rbdyn_urdf.URDFParserResult&)
