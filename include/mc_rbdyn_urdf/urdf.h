#ifndef _H_MCRBDYNURDF_H_
#define _H_MCRBDYNURDF_H_

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
  std::map< int, std::vector<double> > lower;
  std::map< int, std::vector<double> > upper;
  std::map< int, std::vector<double> > velocity;
  std::map< int, std::vector<double> > torque;
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
  std::map<int, std::vector<mc_rbdyn_urdf::Visual>> visual;
  std::map<int, sva::PTransformd> collision_tf;
};

MCRBDYNURDF_API std::vector<double> attrToList(const tinyxml2::XMLElement & dom, const std::string & attr, const std::vector<double> & def = {});

MCRBDYNURDF_API Eigen::Vector3d attrToVector(const tinyxml2::XMLElement & dom, const std::string & attr, const Eigen::Vector3d & def = Eigen::Vector3d(0,0,0));

MCRBDYNURDF_API Eigen::Matrix3d RPY(const double & r, const double & p, const double & y);

MCRBDYNURDF_API rbd::Joint::Type rbdynFromUrdfJoint(const std::string & type);

MCRBDYNURDF_API sva::PTransformd originFromTag(const tinyxml2::XMLElement & root, const std::string & tagName);
MCRBDYNURDF_API sva::PTransformd originFromTag(const tinyxml2::XMLElement * dom);

MCRBDYNURDF_API URDFParserResult rbdyn_from_urdf(const std::string & content, bool fixed = true, const std::vector<std::string> & filteredLinksIn = {}, bool transformInertia = true, const std::string & baseLinkIn = "", bool withVirtualLinks = true);

}

#endif
