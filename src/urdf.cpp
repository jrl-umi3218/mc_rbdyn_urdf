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

#include <mc_rbdyn_urdf/urdf.h>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <algorithm>
#include <iostream>
#include <cmath>
#include <ciso646>

#include "safetinyxml.hxx"

namespace mc_rbdyn_urdf
{

std::vector<double> attrToList(const tinyxml2::XMLElement & dom, const std::string & attr, const std::vector<double> & def)
{
  std::vector<double> res;
  const char * attrTxt = dom.Attribute(attr.c_str());
  if(attrTxt)
  {
    std::stringstream ss;
    ss << attrTxt;
    double tmp;
    while(ss.good())
    {
      ss >> tmp;
      if(!ss.fail())
      {
        res.push_back(tmp);
      }
    }
  }
  else
  {
    res = def;
  }
  return res;
}

Eigen::Vector3d attrToVector(const tinyxml2::XMLElement & dom, const std::string & attr, const Eigen::Vector3d & def)
{
  Eigen::Vector3d res = def;
  std::vector<double> vec = attrToList(dom, attr, {res(0), res(1), res(2)});
  if(vec.size() == 3)
  {
    res(0) = vec[0];
    res(1) = vec[1];
    res(2) = vec[2];
  }
  return res;
}

Eigen::Matrix3d RPY(const double & r, const double & p, const double & y)
{
  double ca1 = cos(y);
  double sa1 = sin(y);
  double cb1 = cos(p);
  double sb1 = sin(p);
  double cc1 = cos(r);
  double sc1 = sin(r);
  Eigen::Matrix3d m;
  m << ca1*cb1, ca1*sb1*sc1 - sa1*cc1, ca1*sb1*cc1 + sa1*sc1,
       sa1*cb1,  sa1*sb1*sc1 + ca1*cc1, sa1*sb1*cc1 - ca1*sc1,
       -sb1,  cb1*sc1, cb1*cc1;
  return m.transpose();
}

Eigen::Matrix3d RPY(const std::vector<double> rpy)
{
  if(rpy.size() != 3)
  {
    std::cerr << "Cannot convert RPY vector of size " << rpy.size() << " to matrix" << std::endl;
    throw(std::runtime_error("Bad vector"));
  }
  return RPY(rpy[0], rpy[1], rpy[2]);
}

inline Eigen::Matrix3d readInertia(const tinyxml2::XMLElement & dom)
{
  Eigen::Matrix3d m;
  m << dom.DoubleAttribute("ixx"), dom.DoubleAttribute("ixy"), dom.DoubleAttribute("ixz"),
       dom.DoubleAttribute("ixy"), dom.DoubleAttribute("iyy"), dom.DoubleAttribute("iyz"),
       dom.DoubleAttribute("ixz"), dom.DoubleAttribute("iyz"), dom.DoubleAttribute("izz");
  return m;
}

rbd::Joint::Type rbdynFromUrdfJoint(const std::string & type, bool hasSphericalSuffix = false)
{
  if(type == "revolute")
    return rbd::Joint::Rev;
  else if(type == "prismatic")
    return rbd::Joint::Prism;
  else if(type == "continuous")
    return rbd::Joint::Rev;
  else if(type == "floating")
  {
    if (hasSphericalSuffix)
      return rbd::Joint::Spherical;
    else
      return rbd::Joint::Free;
  }
  else if(type == "ball")
    return rbd::Joint::Spherical;
  else if(type == "fixed")
    return rbd::Joint::Fixed;
  std::cerr << "Unknown type in URDF " << type << std::endl;
  std::cerr << "Conversion will default to fixed" << std::endl;
  return rbd::Joint::Fixed;
}

sva::PTransformd originFromTag(const tinyxml2::XMLElement & root, const std::string & tagName)
{
  return originFromTag(root.FirstChildElement(tagName.c_str()));
}

sva::PTransformd originFromTag(const tinyxml2::XMLElement *dom)
{
  sva::PTransformd tf = sva::PTransformd::Identity();

  if(dom)
  {
    const tinyxml2::XMLElement * originDom = dom->FirstChildElement("origin");
    if(originDom)
    {
      Eigen::Vector3d T = attrToVector(*originDom, "xyz", Eigen::Vector3d(0,0,0));
      Eigen::Matrix3d R = RPY(attrToList(*originDom, "rpy", {0.0, 0.0, 0.0}));
      tf = sva::PTransformd(R, T);
    }
  }

  return tf;
}

std::string parseMultiBodyGraphFromURDF(URDFParserResult& res, const std::string & content, const std::vector<std::string> & filteredLinksIn, bool transformInertia, const std::string & baseLinkIn, bool withVirtualLinks, const std::string sphericalSuffix)
{
  tinyxml2::XMLDocument doc;
  doc.Parse(content.c_str());
  tinyxml2::XMLElement * robot = doc.FirstChildElement("robot");
  if(!robot)
  {
    std::cerr << "No robot tag in the URDF, parsing will stop now" << std::endl;
    return "";
  }
  std::vector<MaybeElement> links;
  std::vector<std::string> filteredLinks = filteredLinksIn;
  // Extract link elements from the document, remove filtered links
  {
    MaybeElement link = MaybeElement::right(robot) >> firstChildElement("link");
    while(link)
    {
      std::string linkName = link >> attribute("name");
      if(std::find(filteredLinks.begin(), filteredLinks.end(), linkName) == filteredLinks.end())
      {
        if(not withVirtualLinks)
        {
          if(link >> firstChildElement("inertial"))
          {
            links.push_back(link);
          }
          else
          {
            filteredLinks.push_back(linkName);
          }
        }
        else
        {
          links.push_back(link);
        }
      }
      link = link >> nextSiblingElement("link");
    }
  }

  if(links.size() == 0)
  {
    std::cerr << "Failed to extract any link information from the URDF, parsing will stop now" << std::endl;
    return "";
  }

  std::string baseLink = baseLinkIn == "" ? links[0] >> attribute("name") : baseLinkIn;

  for(MaybeElement linkDom : links)
  {
    std::string linkName = linkDom >> attribute("name");
    double mass = 0.0;
    Eigen::Vector3d com = Eigen::Vector3d::Zero();
    std::vector<double> comRPY = {0.0, 0.0, 0.0};
    Eigen::Matrix3d inertia_o = Eigen::Matrix3d::Zero();

    MaybeElement inertialDom  = linkDom >> firstChildElement("inertial");
    if(inertialDom)
    {
      MaybeElement originDom = inertialDom >> firstChildElement("origin");
      MaybeElement massDom = inertialDom >> firstChildElement("mass");
      MaybeElement inertiaDom = inertialDom >> firstChildElement("inertia");

      if(originDom)
      {
        com = attrToVector(*originDom, "xyz", Eigen::Vector3d(0,0,0));
        comRPY = attrToList(*originDom, "rpy", {0.0, 0.0, 0.0});
      }
      Eigen::Matrix3d comFrame = RPY(comRPY);
      mass = inertialDom >> firstChildElement("mass") >> doubleAttribute("value");
      Eigen::Matrix3d inertia = readInertia(*inertiaDom);
      if(transformInertia)
      {
        inertia_o = sva::inertiaToOrigin(inertia, mass, com, comFrame);
      }
      else
      {
        inertia_o = inertia;
      }
    }

    // Parse all visual tags. There may be several per link
    for (MaybeElement child = linkDom >> firstChildElement("visual");
         child; child = child >> nextSiblingElement("visual"))
    {
      Visual v;
      MaybeElement geometryDom = child >> firstChildElement("geometry");
      if (geometryDom)
      {
        MaybeElement meshDom = geometryDom >> firstChildElement("mesh");
        if (meshDom)
        {
          v.origin = originFromTag(child);
          v.geometry.type = Geometry::Type::MESH;
          auto& mesh = boost::get<Geometry::Mesh>(v.geometry.data);
          mesh.filename = (meshDom >> attribute("filename")).fromRight();
          // Optional scale, defaults to 1.
          mesh.scale = meshDom >> doubleAttribute("scale", 1.);
        }
        else
        {
          std::cerr << "Warning: only mesh geometry is supported, visual element has been ignored" << std::endl;
        }
        MaybeString name = child >> attribute("name");
        if(name) v.name = name.fromRight();
        res.visual[linkName].push_back(v);
      }
    }

    // FIXME! Just like visual tags, there can be several collision tags!
    res.collision_tf[linkName] = originFromTag(*linkDom, "collision");

    rbd::Body b(mass, com, inertia_o, linkName);
    res.mbg.addBody(b);
  }

  std::vector<MaybeElement> joints;
  // Extract joint elements from the document, remove joints that link with filtered links
  {
    MaybeElement joint = MaybeElement::right(robot) >> firstChildElement("joint");
    while(joint)
    {
      std::string parent_link = joint >> firstChildElement("parent") >> attribute("link");
      std::string child_link = joint >> firstChildElement("child") >> attribute("link");
      if(std::find(filteredLinks.begin(), filteredLinks.end(), child_link) == filteredLinks.end() &&
         std::find(filteredLinks.begin(), filteredLinks.end(), parent_link) == filteredLinks.end())
      {
        joints.push_back(joint);
      }
      joint = joint >> nextSiblingElement("joint");
    }
  }

  for(MaybeElement& jointDom : joints)
  {
    std::string jointName = jointDom >> attribute("name");
    std::string jointType = jointDom >> attribute("type");

    // Static transformation
    sva::PTransformd staticTransform = sva::PTransformd::Identity();
    MaybeElement originDom = jointDom >> firstChildElement("origin");
    if(originDom)
    {
      Eigen::Vector3d staticT = attrToVector(*originDom, "xyz", Eigen::Vector3d(0,0,0));
      Eigen::Matrix3d staticR = RPY(attrToList(*originDom, "rpy", {0.0, 0.0, 0.0}));
      staticTransform = sva::PTransformd(staticR, staticT);
    }

    // Read the joint axis
    Eigen::Vector3d axis = Eigen::Vector3d::UnitZ();
    tinyxml2::XMLElement * axisDom = jointDom >> firstChildElement("axis");
    if(axisDom)
    {
      axis = attrToVector(*axisDom, "xyz").normalized();
    }
    rbd::Joint::Type type = rbdynFromUrdfJoint(jointType, (jointName.length() >= sphericalSuffix.length()
                            && jointName.substr(jointName.length() - sphericalSuffix.length(), sphericalSuffix.length()) == sphericalSuffix));

    std::string jointParent = jointDom >> firstChildElement("parent") >> attribute("link");

    std::string jointChild = jointDom >> firstChildElement("child") >> attribute("link");

    rbd::Joint j(type, axis, true, jointName);

    // Check if this is a mimic joint
    MaybeElement mimicDom = jointDom >> firstChildElement("mimic");
    if(mimicDom)
    {
      std::string mimicJoint = mimicDom >> attribute("joint");
      double multiplier = mimicDom >> doubleAttribute("multiplier", 1.0);
      double offset = mimicDom >> doubleAttribute("offset", 0.0);
      j.makeMimic(mimicJoint, multiplier, offset);
    }

    res.mbg.addJoint(j);

    res.mbg.linkBodies(jointParent, staticTransform, jointChild, sva::PTransformd::Identity(), jointName);

    // Articular limit
    std::vector<double> lower(static_cast<size_t>(j.dof()), -INFINITY);
    std::vector<double> upper(static_cast<size_t>(j.dof()), INFINITY);
    std::vector<double> effort(static_cast<size_t>(j.dof()), INFINITY);
    std::vector<double> velocity(static_cast<size_t>(j.dof()), INFINITY);

    MaybeElement limitDom = jointDom >> firstChildElement("limit");
    if(limitDom && j.type() != rbd::Joint::Fixed)
    {
      if(jointType != "continuous")
      {
        lower = attrToList(*limitDom, "lower");
        upper = attrToList(*limitDom, "upper");
      }
      effort = attrToList(*limitDom, "effort");
      velocity = attrToList(*limitDom, "velocity");
    }
    auto check_limit = [&j](const std::string & name, const std::vector<double> & limit)
    {
      if(limit.size() != static_cast<size_t>(j.dof()))
      {
        std::cerr << "Joint " << name << " limit for " << j.name() << ": size missmatch, expected: " << j.dof() << ", got: " << limit.size() << std::endl;
      }
    };
    check_limit("lower", lower);
    check_limit("upper", upper);
    check_limit("effort", effort);
    check_limit("velocity", velocity);
    res.limits.lower[jointName] = lower;
    res.limits.upper[jointName] = upper;
    res.limits.torque[jointName] = effort;
    res.limits.velocity[jointName] = velocity;
  }

  return baseLink;
}

URDFParserResult rbdyn_from_urdf(const std::string & content, bool fixed, const std::vector<std::string> & filteredLinksIn, bool transformInertia, const std::string & baseLinkIn, bool withVirtualLinks, const std::string sphericalSuffix)
{
  URDFParserResult res;

  std::string baseLink = parseMultiBodyGraphFromURDF(res, content, filteredLinksIn, transformInertia, baseLinkIn, withVirtualLinks, sphericalSuffix);

  if(!res.mbg.nrNodes())
  {
    throw std::runtime_error("Unable to extract valid robot");
  }

  res.mb = res.mbg.makeMultiBody(baseLink, fixed);
  res.mbc = rbd::MultiBodyConfig(res.mb);
  res.mbc.zero(res.mb);

  rbd::forwardKinematics(res.mb, res.mbc);
  rbd::forwardVelocity(res.mb, res.mbc);

  return res;
}

}
