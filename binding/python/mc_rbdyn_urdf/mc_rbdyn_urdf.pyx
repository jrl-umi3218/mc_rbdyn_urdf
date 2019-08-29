# distutils: language = c++

#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cimport c_mc_rbdyn_urdf
cimport c_mc_rbdyn_urdf_private

cimport eigen.eigen as eigen
cimport sva.sva as sva
cimport rbdyn.rbdyn as rbdyn

from cython.operator cimport dereference as deref
from libcpp.map cimport map as cppmap
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

global __EXPERT_MODE__
__EXPERT_MODE__ = False

def EXPERT_MODE(value = None):
  global __EXPERT_MODE__
  if value is not None:
    __EXPERT_MODE__ = value
  else:
    return __EXPERT_MODE__

cdef class Limits(object):
  def __cinit__(self, *args):
    pass
  property lower:
    def __get__(self):
      return self.impl.lower
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      if any(map(lambda k: isinstance(k, unicode), value)):
        nvalue = {}
        for k, v in value.items():
          if isinstance(k, unicode):
            nvalue[k.encode(u'ascii')] = v
          else:
            nvalue[k] = v
        self.impl.lower = nvalue
      else:
        self.impl.lower = value
  property upper:
    def __get__(self):
      return self.impl.upper
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      if any(map(lambda k: isinstance(k, unicode), value)):
        nvalue = {}
        for k, v in value.items():
          if isinstance(k, unicode):
            nvalue[k.encode(u'ascii')] = v
          else:
            nvalue[k] = v
        self.impl.upper = nvalue
      else:
        self.impl.upper = value
  property velocity:
    def __get__(self):
      return self.impl.velocity
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      if any(map(lambda k: isinstance(k, unicode), value)):
        nvalue = {}
        for k, v in value.items():
          if isinstance(k, unicode):
            nvalue[k.encode(u'ascii')] = v
          else:
            nvalue[k] = v
        self.impl.velocity = nvalue
      else:
        self.impl.velocity = value
  property torque:
    def __get__(self):
      return self.impl.torque
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      if any(map(lambda k: isinstance(k, unicode), value)):
        nvalue = {}
        for k, v in value.items():
          if isinstance(k, unicode):
            nvalue[k.encode(u'ascii')] = v
          else:
            nvalue[k] = v
        self.impl.torque = nvalue
      else:
        self.impl.torque = value

cdef Limits LimitsFromC(const c_mc_rbdyn_urdf.Limits & lim):
    cdef Limits ret = Limits()
    ret.impl = lim
    return ret

cdef class GeometryMesh(object):
  def __cinit__(self, *args):
    pass
  property filename:
    def __get__(self):
      return self.impl.filename
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      if isinstance(value, unicode):
        value = value.encode(u'ascii')
      self.impl.filename = value
  property scale:
    def __get__(self):
      return self.impl.scale
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      self.impl.scale = value
  def __richcmp__(GeometryMesh self, GeometryMesh other, int op):
    if op == 2:
      return self.impl.filename == other.impl.filename and self.impl.scale == other.impl.scale
    elif op == 3:
      return self.impl.filename != other.impl.filename and self.impl.scale != other.impl.scale
    else:
      raise NotImplementedError("This comparison is not supported")

cdef GeometryMesh GeometryMeshFromC(const c_mc_rbdyn_urdf.GeometryMesh & m):
    cdef GeometryMesh ret = GeometryMesh()
    ret.impl = m
    return ret

cdef class GeometryBox(object):
  def __cinit__(self, *args):
    pass
  property size:
    def __get__(self):
      return self.impl.size
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      self.impl.size = value
  def __richcmp__(GeometryBox self, GeometryBox other, int op):
    if op == 2:
      return self.impl.size == other.impl.size
    elif op == 3:
      return self.impl.size != other.impl.size
    else:
      raise NotImplementedError("This comparison is not supported")

cdef GeometryBox GeometryBoxFromC(const c_mc_rbdyn_urdf.GeometryBox & m):
    cdef GeometryBox ret = GeometryBox()
    ret.impl = m
    return ret

cdef class GeometryCylinder(object):
  def __cinit__(self, *args):
    pass
  property radius:
    def __get__(self):
      return self.impl.radius
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      self.impl.radius = value
  property length:
    def __get__(self):
      return self.impl.length
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      self.impl.length = value
  def __richcmp__(GeometryCylinder self, GeometryCylinder other, int op):
    if op == 2:
      return self.impl.radius == other.radius and self.impl.length == other.impl.length
    elif op == 3:
      return self.impl.radius != other.radius and self.impl.length != other.impl.length
    else:
      raise NotImplementedError("This comparison is not supported")

cdef GeometryCylinder GeometryCylinderFromC(const c_mc_rbdyn_urdf.GeometryCylinder & m):
    cdef GeometryCylinder ret = GeometryCylinder()
    ret.impl = m
    return ret

cdef class GeometrySphere(object):
  def __cinit__(self, *args):
    pass
  property radius:
    def __get__(self):
      return self.impl.radius
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      self.impl.radius = value
  def __richcmp__(GeometrySphere self, GeometrySphere other, int op):
    if op == 2:
      return self.impl.radius == other.impl.radius
    elif op == 3:
      return self.impl.radius != other.impl.radius
    else:
      raise NotImplementedError("This comparison is not supported")

cdef GeometrySphere GeometrySphereFromC(const c_mc_rbdyn_urdf.GeometrySphere & m):
    cdef GeometrySphere ret = GeometrySphere()
    ret.impl = m
    return ret

cdef class Geometry(object):
  MESH = c_mc_rbdyn_urdf.GeometryMESH
  BOX = c_mc_rbdyn_urdf.GeometryBOX
  CYLINDER = c_mc_rbdyn_urdf.GeometryCYLINDER
  SPHERE = c_mc_rbdyn_urdf.GeometrySPHERE
  UNKNOWN = c_mc_rbdyn_urdf.GeometryUNKNOWN
  def __cinit__(self, *args):
    pass
  property type:
    def __get__(self):
      return self.impl._type
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      assert(value >= c_mc_rbdyn_urdf.GeometryBOX and value <= c_mc_rbdyn_urdf.GeometryUNKNOWN)
      self.impl._type = <c_mc_rbdyn_urdf.GeometryType>value
  def __set_geom_mesh(self, GeometryMesh value):
    assert(self.impl._type == c_mc_rbdyn_urdf.GeometryMESH)
    c_mc_rbdyn_urdf_private.setMesh(self.impl, value.impl)
  def __set_geom_box(self, GeometryBox value):
    assert(self.impl._type == c_mc_rbdyn_urdf.GeometryBOX)
    c_mc_rbdyn_urdf_private.setBox(self.impl, value.impl)
  def __set_geom_cylinder(self, GeometryCylinder value):
    assert(self.impl._type == c_mc_rbdyn_urdf.GeometryCYLINDER)
    c_mc_rbdyn_urdf_private.setCylinder(self.impl, value.impl)
  def __set_geom_sphere(self, GeometrySphere value):
    assert(self.impl._type == c_mc_rbdyn_urdf.GeometrySPHERE)
    c_mc_rbdyn_urdf_private.setSphere(self.impl, value.impl)
  property data:
    def __get__(self):
      if self.impl._type == c_mc_rbdyn_urdf.GeometryMESH:
        return GeometryMeshFromC(c_mc_rbdyn_urdf_private.getMesh(self.impl))
      elif self.impl._type == c_mc_rbdyn_urdf.GeometryBOX:
        return GeometryBoxFromC(c_mc_rbdyn_urdf_private.getBox(self.impl))
      elif self.impl._type == c_mc_rbdyn_urdf.GeometryCYLINDER:
        return GeometryCylinderFromC(c_mc_rbdyn_urdf_private.getCylinder(self.impl))
      elif self.impl._type == c_mc_rbdyn_urdf.GeometrySPHERE:
        return GeometrySphereFromC(c_mc_rbdyn_urdf_private.getSphere(self.impl))
      else:
        print "No data in the Geometry you are accessing"
        return None
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      if isinstance(value, GeometryMesh):
        self.__set_geom_mesh(value)
      elif isinstance(value, GeometryBox):
        self.__set_geom_box(value)
      elif isinstance(value, GeometryCylinder):
        self.__set_geom_cylinder(value)
      elif isinstance(value, GeometrySphere):
        self.__set_geom_sphere(value)
      else:
        raise TypeError("Cannot set geometry data with this value")
  def __richcmp__(Geometry self, Geometry other, int op):
    if op == 2:
      return self.impl._type == other.impl._type and self.data == other.data
    elif op == 3:
      return self.impl._type != other.impl._type and self.data != other.data
    else:
      raise NotImplementedError("This comparison is not supported")

cdef Geometry GeometryFromC(const c_mc_rbdyn_urdf.Geometry & g):
    cdef Geometry ret = Geometry()
    ret.impl = g
    return ret

cdef class Visual(object):
  def __cinit__(self, *args):
    pass
  property name:
    def __get__(self):
      return self.impl.name
    def __set__(self, value):
      assert(__EXPERT_MODE__)
      self.impl.name = value
  property origin:
    def __get__(self):
      return sva.PTransformdFromC(self.impl.origin)
    def __set__(self, sva.PTransformd value):
      assert(__EXPERT_MODE__)
      self.impl.origin = deref(value.impl)
  property geometry:
    def __get__(self):
      return GeometryFromC(self.impl.geometry)
    def __set__(self, Geometry value):
      assert(__EXPERT_MODE__)
      self.impl.geometry = value.impl
  def __richcmp__(Visual self, Visual other, int op):
    if op == 2:
      return self.impl.name == other.impl.name and self.impl.origin == other.impl.origin and self.geometry == other.geometry
    elif op == 3:
      return self.impl.name != other.impl.name and self.impl.origin != other.impl.origin and self.geometry != other.geometry
    else:
      raise NotImplementedError("This comparison is not supported")

cdef Visual VisualFromC(const c_mc_rbdyn_urdf.Visual& v):
    cdef Visual ret = Visual()
    ret.impl = v
    return ret

cdef class URDFParserResult(object):
  def __cinit__(self, *args):
    pass
  property mb:
    def __get__(self):
      return rbdyn.MultiBodyFromC(self.impl.mb, copy = False)
  property mbc:
    def __get__(self):
      return rbdyn.MultiBodyConfigFromC(self.impl.mbc, copy = False)
  property mbg:
    def __get__(self):
      return rbdyn.MultiBodyGraphFromC(self.impl.mbg, copy = False)
  property limits:
    def __get__(self):
      return LimitsFromC(self.impl.limits)
  property visual:
    def __get__(self):
      res = {}
      for it in self.impl.visual:
        res[it.first] = []
        for v in it.second:
          res[it.first].append(VisualFromC(v))
      return res
  property collision_tf:
    def __get__(self):
      res = {}
      for it in self.impl.collision_tf:
        res[it.first] = sva.PTransformdFromC(it.second)
      return res

cdef URDFParserResult URDFParserResultFromC(const c_mc_rbdyn_urdf.URDFParserResult & u):
    cdef URDFParserResult res = URDFParserResult()
    res.impl = u
    return res

def rbdyn_from_urdf(content, fixed = True, filteredLinks = [], transformInertia = True, baseLink = "Root", withVirtualLinks = True):
  if isinstance(content, unicode):
    content = content.encode(u'ascii')
  if isinstance(baseLink, unicode):
    baseLink = baseLink.encode(u'ascii')
  filteredLinks = [ fL.encode(u'ascii') if isinstance(fL, unicode) else fL for fL in filteredLinks ]
  return URDFParserResultFromC(c_mc_rbdyn_urdf.rbdyn_from_urdf(content, fixed, filteredLinks, transformInertia, baseLink, withVirtualLinks))
