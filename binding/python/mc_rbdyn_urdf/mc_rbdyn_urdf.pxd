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
