#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

ADD_CYTHON_BINDINGS(mc_rbdyn_urdf
  TARGETS mc_rbdyn_urdf
  EXPORT_SOURCES mc_rbdyn_urdf/__init__.py mc_rbdyn_urdf/c_mc_rbdyn_urdf.pxd mc_rbdyn_urdf/mc_rbdyn_urdf.pxd
  PRIVATE_SOURCES include/mc_rbdyn_urdf_wrapper.hpp mc_rbdyn_urdf/c_mc_rbdyn_urdf_private.pxd
                  mc_rbdyn_urdf/mc_rbdyn_urdf.pyx tests/test_mc_rbdyn_urdf.py
)
