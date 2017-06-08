/* Copyright 2015-2017 CNRS-UM LIRMM, CNRS-AIST JRL
 *
 * This file is part of mc_rbdyn_urdf.
 *
 * mc_rbdyn_urdf is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * mc_rbdyn_urdf is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with mc_rbdyn_urdf.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <mc_rbdyn_urdf/urdf.h>

namespace mc_rbdyn_urdf
{
  const Geometry::Mesh& getMesh(const Geometry & geom)
  {
    return boost::get<Geometry::Mesh>(geom.data);
  }

  const Geometry::Box& getBox(const Geometry & geom)
  {
    return boost::get<Geometry::Box>(geom.data);
  }

  const Geometry::Cylinder& getCylinder(const Geometry & geom)
  {
    return boost::get<Geometry::Cylinder>(geom.data);
  }

  const Geometry::Sphere& getSphere(const Geometry & geom)
  {
    return boost::get<Geometry::Sphere>(geom.data);
  }

  void setMesh(Geometry & geom, const Geometry::Mesh & data)
  {
    geom.data = data;
  }

  void setBox(Geometry & geom, const Geometry::Box & data)
  {
    geom.data = data;
  }

  void setCylinder(Geometry & geom, const Geometry::Cylinder & data)
  {
    geom.data = data;
  }

  void setSphere(Geometry & geom, const Geometry::Sphere & data)
  {
    geom.data = data;
  }
}
