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

set(SOURCES
  urdf.cpp
)
set(HEADERS
  ../include/mc_rbdyn_urdf/api.h
  ../include/mc_rbdyn_urdf/urdf.h
)

add_library(mc_rbdyn_urdf SHARED ${SOURCES} ${HEADERS})
pkg_config_use_dependency(mc_rbdyn_urdf RBDyn)
pkg_config_use_dependency(mc_rbdyn_urdf tinyxml2)

set_target_properties(mc_rbdyn_urdf PROPERTIES SOVERSION 1 VERSION 1.1.0)
target_compile_definitions(mc_rbdyn_urdf PRIVATE "-DMCRBDYNURDF_BUILDING")

set(INSTALL_PATH include/mc_rbdyn_urdf)
install(FILES ${HEADERS} DESTINATION ${INSTALL_PATH})
install(
  TARGETS mc_rbdyn_urdf
  EXPORT "${TARGETS_EXPORT_NAME}"
  LIBRARY DESTINATION "lib"
  ARCHIVE DESTINATION "lib"
  RUNTIME DESTINATION "bin"
  INCLUDES DESTINATION "${INCLUDE_INSTALL_DIR}"
)
