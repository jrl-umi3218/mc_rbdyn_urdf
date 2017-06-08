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

#include <SpaceVecAlg/SpaceVecAlg>
#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>
#include <Eigen/Core>
#include <tinyxml2.h>
#include <string>

#include <mc_rbdyn_urdf/api.h>
#include <boost/variant.hpp>

namespace mc_rbdyn_urdf
{

struct MCRBDYNURDF_API Limits
{
public:
  std::map< std::string, std::vector<double> > lower;
  std::map< std::string, std::vector<double> > upper;
  std::map< std::string, std::vector<double> > velocity;
  std::map< std::string, std::vector<double> > torque;
};

struct MCRBDYNURDF_API Geometry
{
 public:
  struct Mesh
  {
    Mesh() : scale(1) {}
    std::string filename;
    double scale;
  };
  struct Box
  {
    Box() : size(0.)  {}
    double size;
  };
  struct Cylinder
  {
    Cylinder() : radius(0.), length(0.) {}
    double radius;
    double length;
  };
  struct Sphere
  {
    Sphere() : radius(0.) {}
    double radius;
  };

  enum Type { BOX, CYLINDER, SPHERE, MESH, UNKNOWN };
  Type type;
  boost::variant<Mesh, Box, Cylinder, Sphere> data;

  Geometry() : type(UNKNOWN) {}
};

struct MCRBDYNURDF_API Visual
{
  std::string name;
  sva::PTransformd origin;
  Geometry geometry;
};


struct MCRBDYNURDF_API URDFParserResult
{
public:
  rbd::MultiBody mb;
  rbd::MultiBodyConfig mbc;
  rbd::MultiBodyGraph mbg;
  mc_rbdyn_urdf::Limits limits;
  std::map<std::string, std::vector<mc_rbdyn_urdf::Visual>> visual;
  std::map<std::string, sva::PTransformd> collision_tf;
};

MCRBDYNURDF_API std::vector<double> attrToList(const tinyxml2::XMLElement & dom, const std::string & attr, const std::vector<double> & def = {});

MCRBDYNURDF_API Eigen::Vector3d attrToVector(const tinyxml2::XMLElement & dom, const std::string & attr, const Eigen::Vector3d & def = Eigen::Vector3d(0,0,0));

MCRBDYNURDF_API Eigen::Matrix3d RPY(const double & r, const double & p, const double & y);

MCRBDYNURDF_API rbd::Joint::Type rbdynFromUrdfJoint(const std::string & type);

MCRBDYNURDF_API sva::PTransformd originFromTag(const tinyxml2::XMLElement & root, const std::string & tagName);
MCRBDYNURDF_API sva::PTransformd originFromTag(const tinyxml2::XMLElement * dom);

MCRBDYNURDF_API URDFParserResult rbdyn_from_urdf(const std::string & content, bool fixed = true, const std::vector<std::string> & filteredLinksIn = {}, bool transformInertia = true, const std::string & baseLinkIn = "", bool withVirtualLinks = true, const std::string sphericalSuffix = "_spherical");

MCRBDYNURDF_API std::string parseMultiBodyGraphFromURDF(URDFParserResult& res, const std::string & content, const std::vector<std::string> & filteredLinksIn = {}, bool transformInertia = true, const std::string & baseLinkIn = "", bool withVirtualLinks = true, const std::string sphericalSuffix = "_spherical");

}
