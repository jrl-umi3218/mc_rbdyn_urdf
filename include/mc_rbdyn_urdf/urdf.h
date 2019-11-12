/*
 * Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn_urdf/api.h>

#include <RBDyn/Joint.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyConfig.h>
#include <RBDyn/MultiBodyGraph.h>

#include <SpaceVecAlg/SpaceVecAlg>

#include <Eigen/Core>
#include <string>
#include <tinyxml2.h>

namespace mc_rbdyn_urdf
{

struct MCRBDYNURDF_API Limits
{
public:
  std::map<std::string, std::vector<double>> lower;
  std::map<std::string, std::vector<double>> upper;
  std::map<std::string, std::vector<double>> velocity;
  std::map<std::string, std::vector<double>> torque;
};

struct MCRBDYNURDF_API Geometry
{
public:
  struct Mesh
  {
    Mesh() : scale(1) {}
    Mesh(const std::string & filename, double s) : filename(filename), scale(s) {}
    std::string filename;
    double scale;
  };
  struct Box
  {
    Box() : size(0.) {}
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
  struct Unknown
  {
  };

  enum Type
  {
    BOX,
    CYLINDER,
    SPHERE,
    MESH,
    UNKNOWN
  };
  Type type = UNKNOWN;
  union GeometryT
  {
    GeometryT() : u() {}
    Mesh m;
    Box b;
    Cylinder c;
    Sphere s;
    Unknown u;
    ~GeometryT() {}
  };
  const GeometryT & data() const { return data_; }

  /** Set geometry to the provided box */
  void box(const Box & b);

  /** Set geometry to the provided cylinder */
  void cylinder(const Cylinder & c);

  /** Set geometry to the provided sphere */
  void sphere(const Sphere & s);

  /** Set geometry to the provided mesh */
  void mesh(const Mesh & m);

  /** Set geometry to unknown */
  void unknown();

  Geometry() : type(UNKNOWN) {}

  ~Geometry();

  Geometry(const Geometry & rhs);
  Geometry& operator=(const Geometry & rhs);
private:
  GeometryT data_;
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

MCRBDYNURDF_API std::vector<double> attrToList(const tinyxml2::XMLElement & dom,
                                               const std::string & attr,
                                               const std::vector<double> & def = {});

MCRBDYNURDF_API Eigen::Vector3d attrToVector(const tinyxml2::XMLElement & dom,
                                             const std::string & attr,
                                             const Eigen::Vector3d & def = Eigen::Vector3d(0, 0, 0));

MCRBDYNURDF_API Eigen::Matrix3d RPY(const double & r, const double & p, const double & y);

MCRBDYNURDF_API rbd::Joint::Type rbdynFromUrdfJoint(const std::string & type);

MCRBDYNURDF_API sva::PTransformd originFromTag(const tinyxml2::XMLElement & root, const std::string & tagName);

MCRBDYNURDF_API sva::PTransformd originFromTag(const tinyxml2::XMLElement & root, const std::string & tagName);

MCRBDYNURDF_API URDFParserResult rbdyn_from_urdf(const std::string & content,
                                                 bool fixed = true,
                                                 const std::vector<std::string> & filteredLinksIn = {},
                                                 bool transformInertia = true,
                                                 const std::string & baseLinkIn = "",
                                                 bool withVirtualLinks = true,
                                                 const std::string sphericalSuffix = "_spherical");

MCRBDYNURDF_API std::string parseMultiBodyGraphFromURDF(URDFParserResult & res,
                                                        const std::string & content,
                                                        const std::vector<std::string> & filteredLinksIn = {},
                                                        bool transformInertia = true,
                                                        const std::string & baseLinkIn = "",
                                                        bool withVirtualLinks = true,
                                                        const std::string sphericalSuffix = "_spherical");

} // namespace mc_rbdyn_urdf
