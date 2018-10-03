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

cmake_minimum_required(VERSION 2.8)

include(cmake/base.cmake)
include(cmake/boost.cmake)
include(cmake/eigen.cmake)
include(cmake/msvc-specific.cmake)

set(PROJECT_NAME mc_rbdyn_urdf)
set(PROJECT_DESCRIPTION "...")
set(PROJECT_URL "https://github.com/jrl-umi3218/mc_rbdyn_urdf")
set(PROJECT_DEBUG_POSTFIX "_d")

set(DOXYGEN_USE_MATHJAX "YES")
set(CXX_DISABLE_WERROR True)
setup_project()

option(PYTHON_BINDING "Generate python binding." ON)
if(WIN32)
  set(PYTHON_BINDING_USER_INSTALL_DEFAULT ON)
else()
  set(PYTHON_BINDING_USER_INSTALL_DEFAULT OFF)
endif()
option(PYTHON_BINDING_USER_INSTALL "Install the Python bindings in user space" ${PYTHON_BINDING_USER_INSTALL_DEFAULT})
option(DISABLE_TESTS "Disable unit tests." OFF)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++0x -pedantic")
endif()

search_for_eigen()

add_required_dependency("SpaceVecAlg")
add_required_dependency("RBDyn" "RBDyn_d")
add_required_dependency("tinyxml2")

# For MSVC, set local environment variable to enable finding the built dll
# of the main library when launching ctest with RUN_TESTS
if(MSVC)
  set(CMAKE_MSVCIDE_RUN_PATH "\$(SolutionDir)/src/\$(Configuration)")
endif()

add_subdirectory(src)

if(NOT ${DISABLE_TESTS})
  add_subdirectory(tests)
endif()

if(${PYTHON_BINDING})
 add_subdirectory(binding/python)
endif()

pkg_config_append_libs(mc_rbdyn_urdf)
setup_project_finalize()
setup_project_package_finalize()

