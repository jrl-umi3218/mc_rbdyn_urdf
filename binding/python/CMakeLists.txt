# Copyright 2015-2017 CNRS-UM LIRMM, CNRS-AIST JRL
#
# This file is part of mc_rbdyn_urdf.
#
# mc_rbdyn_urdf is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# mc_rbdyn_urdf is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with mc_rbdyn_urdf.  If not, see <http://www.gnu.org/licenses/>.

macro(GET_MC_RBDYN_URDF_PROPERTY PROPERTY)
  get_target_property(MC_RBDYN_URDF_${PROPERTY} mc_rbdyn_urdf ${PROPERTY})
  if(NOT MC_RBDYN_URDF_${PROPERTY})
    set(MC_RBDYN_URDF_${PROPERTY} "")
  endif()
endmacro()
GET_MC_RBDYN_URDF_PROPERTY(COMPILE_FLAGS)
GET_MC_RBDYN_URDF_PROPERTY(INCLUDE_DIRECTORIES)
GET_MC_RBDYN_URDF_PROPERTY(LINK_FLAGS)
GET_MC_RBDYN_URDF_PROPERTY(LINK_LIBRARIES)
GET_MC_RBDYN_URDF_PROPERTY(LOCATION)
configure_file(setup.in.py ${CMAKE_CURRENT_SOURCE_DIR}/setup.py)

# Build the bindings locally at build time for test purposes
add_custom_target(mc_rbdyn_urdf-python-bindings ALL
  COMMAND python setup.py build_ext --inplace
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Generating local mc_rbdyn_urdf Python bindings"
  DEPENDS include/mc_rbdyn_urdf_wrapper.hpp mc_rbdyn_urdf/c_mc_rbdyn_urdf.pxd mc_rbdyn_urdf/mc_rbdyn_urdf.pxd mc_rbdyn_urdf/mc_rbdyn_urdf.pyx
)

if(NOT ${DISABLE_TESTS})
  add_custom_target(test-mc_rbdyn_urdf-python-bindings
    COMMAND nosetests
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Test mc_rbdyn_urdf Python bindings"
  )
  add_test(NAME PythonBindingsTest
    COMMAND nosetests
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

set(PIP_EXTRA_OPTIONS "")
if(${PYTHON_BINDING_USER_INSTALL})
  set(PIP_EXTRA_OPTIONS "--user")
endif()
# Install the bindings
add_custom_target(install-mc_rbdyn_urdf-python-bindings
  COMMAND pip install . ${PIP_EXTRA_OPTIONS}
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  COMMENT "Install mc_rbdyn_urdf Python bindings"
)
install(CODE "EXECUTE_PROCESS(COMMAND ${CMAKE_MAKE_PROGRAM} install-mc_rbdyn_urdf-python-bindings)")
