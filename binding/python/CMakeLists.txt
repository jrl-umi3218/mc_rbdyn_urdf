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

set(SETUP_LOCATION "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIGURATION>")
if(DEFINED CMAKE_BUILD_TYPE)
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_BUILD_TYPE}")
else()
  foreach(CFG ${CMAKE_CONFIGURATION_TYPES})
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/${CFG}")
  endforeach()
endif()
configure_file(setup.in.py "${CMAKE_CURRENT_BINARY_DIR}/setup.in.py")
FILE(GENERATE
      OUTPUT "${SETUP_LOCATION}/setup.py"
      INPUT  "${CMAKE_CURRENT_BINARY_DIR}/setup.in.py")


# Build the bindings locally at build time for test purposes
add_custom_target(mc_rbdyn_urdf-python-bindings ALL
  COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" python setup.py build_ext --inplace
  COMMENT "Generating local mc_rbdyn_urdf Python bindings"
  DEPENDS include/mc_rbdyn_urdf_wrapper.hpp mc_rbdyn_urdf/c_mc_rbdyn_urdf.pxd mc_rbdyn_urdf/mc_rbdyn_urdf.pxd mc_rbdyn_urdf/mc_rbdyn_urdf.pyx
  SOURCES include/mc_rbdyn_urdf_wrapper.hpp mc_rbdyn_urdf/c_mc_rbdyn_urdf.pxd mc_rbdyn_urdf/mc_rbdyn_urdf.pxd mc_rbdyn_urdf/mc_rbdyn_urdf.pyx
)

if(NOT ${DISABLE_TESTS})
  if(NOT DEFINED PYTHON_DEB_ROOT)
    add_test(NAME PythonBindingsTest
      COMMAND ${CMAKE_COMMAND} -E chdir "$<TARGET_FILE_DIR:mc_rbdyn_urdf>" nosetests ${SETUP_LOCATION}
    )
  endif()
endif()

set(PIP_EXTRA_OPTIONS "")
if(${PYTHON_BINDING_USER_INSTALL})
  set(PIP_EXTRA_OPTIONS "--user")
endif()
# Install the bindings
if(DEFINED PYTHON_DEB_ROOT)
  add_custom_target(install-mc_rbdyn_urdf-python-bindings
    COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" python setup.py install --root=${PYTHON_DEB_ROOT} --install-layout=deb && python3 setup.py install --root=${PYTHON_DEB_ROOT} --install-layout=deb
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Install mc_rbdyn_urdf Python bindings (Debian package)"
  )
else()
  add_custom_target(install-mc_rbdyn_urdf-python-bindings
    COMMAND ${CMAKE_COMMAND} -E chdir "${SETUP_LOCATION}" pip install . ${PIP_EXTRA_OPTIONS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Install mc_rbdyn_urdf Python bindings"
  )
endif()
install(CODE "EXECUTE_PROCESS(COMMAND \"${CMAKE_COMMAND}\" --build \"${CMAKE_BINARY_DIR}\" --config \${CMAKE_INSTALL_CONFIG_NAME} --target install-mc_rbdyn_urdf-python-bindings)")
