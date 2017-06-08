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
