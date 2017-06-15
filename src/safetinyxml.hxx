#pragma once

#include "either.hxx"
#include <tinyxml2.h>

namespace mc_rbdyn_urdf
{
  using MaybeElement = either<std::string, tinyxml2::XMLElement*>;
  using MaybeDouble = either<std::string, double>;
  using MaybeString = either<std::string, std::string>;

  std::function<MaybeElement(tinyxml2::XMLElement*)>
  firstChildElement(const std::string& tag)
  {
    return [&tag](tinyxml2::XMLElement * e)
    {
      auto res = e->FirstChildElement(tag.c_str());
      if(res)
      {
        return MaybeElement::right(res);
      }
      else
      {
        return MaybeElement::left("No child named " + tag);
      }
    };
  }

  std::function<MaybeElement(tinyxml2::XMLElement*)>
  nextSiblingElement(const std::string& tag)
  {
    return [&tag](tinyxml2::XMLElement * e)
    {
      auto res = e->NextSiblingElement(tag.c_str());
      if(res)
      {
        return MaybeElement::right(res);
      }
      else
      {
        return MaybeElement::left("No child named " + tag);
      }
    };
  }

  std::function<MaybeDouble(tinyxml2::XMLElement*)>
  doubleAttribute(const std::string& attr)
  {
    return [attr](tinyxml2::XMLElement* elem)
    {
      return MaybeDouble::right(elem->DoubleAttribute(attr.c_str()));
    };
  }

  std::function<MaybeDouble(tinyxml2::XMLElement*)>
  doubleAttribute(const std::string& attr, double def)
  {
    return [attr, def](tinyxml2::XMLElement* elem)
    {
      double res = def;
      elem->QueryDoubleAttribute(attr.c_str(), &res);
      return MaybeDouble::right(res);
    };
  }

  std::function<MaybeString(tinyxml2::XMLElement*)>
  attribute(const std::string& attr)
  {
    return [attr](tinyxml2::XMLElement* elem)
    {
      auto ret = elem->Attribute(attr.c_str());
      if(ret)
      {
        return MaybeString::right(ret);
      }
      else
      {
        return MaybeString::left("No attribute named " + attr);
      }
    };
  }
}
