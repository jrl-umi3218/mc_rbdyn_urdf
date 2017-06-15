#pragma once

#include "either.hxx"
#include <tinyxml2.h>

namespace mc_rbdyn_urdf
{
  using MaybeElement = either<std::string, tinyxml2::XMLElement*>;
  using MaybeDouble = either<std::string, double>;

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

  std::function<MaybeDouble(tinyxml2::XMLElement*)>
  doubleAttribute(const std::string& attr)
  {
    return [attr](tinyxml2::XMLElement* elem)
    {
      return MaybeDouble::right(elem->DoubleAttribute(attr.c_str()));
    };
  }
}
