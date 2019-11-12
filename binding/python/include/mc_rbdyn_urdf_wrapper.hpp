/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn_urdf/urdf.h>

namespace mc_rbdyn_urdf
{
  const Geometry::Mesh& getMesh(const Geometry & geom)
  {
    return geom.data().m;
  }

  const Geometry::Box& getBox(const Geometry & geom)
  {
    return geom.data().b;
  }

  const Geometry::Cylinder& getCylinder(const Geometry & geom)
  {
    return geom.data().c;
  }

  const Geometry::Sphere& getSphere(const Geometry & geom)
  {
    return geom.data().s;
  }

  void setMesh(Geometry & geom, const Geometry::Mesh & data)
  {
    geom.mesh(data);
  }

  void setBox(Geometry & geom, const Geometry::Box & data)
  {
    geom.box(data);
  }

  void setCylinder(Geometry & geom, const Geometry::Cylinder & data)
  {
    geom.cylinder(data);
  }

  void setSphere(Geometry & geom, const Geometry::Sphere & data)
  {
    geom.sphere(data);
  }
}
