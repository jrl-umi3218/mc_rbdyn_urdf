/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
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
