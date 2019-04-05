#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

from sva.c_sva cimport PTransformd
from rbdyn.c_rbdyn cimport MultiBody, MultiBodyConfig, MultiBodyGraph

from libcpp.map cimport map as cppmap
from libcpp.pair cimport pair
from libcpp.string cimport string
from libcpp.vector cimport vector
from libcpp cimport bool as cppbool

cdef extern from "<mc_rbdyn_urdf/urdf.h>" namespace "mc_rbdyn_urdf":
  cdef cppclass Limits:
    cppmap[string, vector[double]] lower
    cppmap[string, vector[double]] upper
    cppmap[string, vector[double]] velocity
    cppmap[string, vector[double]] torque

  cdef cppclass GeometryMesh "mc_rbdyn_urdf::Geometry::Mesh":
    string filename
    double scale

  cdef cppclass GeometryBox "mc_rbdyn_urdf::Geometry::Box":
    double size

  cdef cppclass GeometryCylinder "mc_rbdyn_urdf::Geometry::Cylinder":
    double radius
    double length

  cdef cppclass GeometrySphere "mc_rbdyn_urdf::Geometry::Sphere":
    double radius

  ctypedef enum GeometryType "mc_rbdyn_urdf::Geometry::Type":
    GeometryBOX "mc_rbdyn_urdf::Geometry::BOX"
    GeometryCYLINDER "mc_rbdyn_urdf::Geometry::CYLINDER"
    GeometrySPHERE "mc_rbdyn_urdf::Geometry::SPHERE"
    GeometryMESH "mc_rbdyn_urdf::Geometry::MESH"
    GeometryUNKNOWN "mc_rbdyn_urdf::Geometry::UNKNOWN"

  cdef cppclass Geometry:
    GeometryType _type "type"
    # data is boost::variant so it is exposed through the wrapper

  cdef cppclass Visual:
    string name
    PTransformd origin
    Geometry geometry

  cdef cppclass URDFParserResult:
    MultiBody mb
    MultiBodyConfig mbc
    MultiBodyGraph mbg
    Limits limits
    cppmap[string, vector[Visual]] visual
    cppmap[string, PTransformd] collision_tf

  URDFParserResult rbdyn_from_urdf(const string&, cppbool, const vector[string]&, cppbool, const string&, cppbool)
