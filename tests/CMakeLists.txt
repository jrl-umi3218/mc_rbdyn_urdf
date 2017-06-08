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

enable_testing()

set(Boost_USE_STATIC_LIBS ON)
set(BOOST_COMPONENTS unit_test_framework)
search_for_boost()
include_directories(${Boost_INCLUDE_DIR})

add_executable(test_mc_rbdyn_urdf test_mc_rbdyn_urdf.cpp)
target_link_libraries(test_mc_rbdyn_urdf ${Boost_LIBRARIES} mc_rbdyn_urdf)
pkg_config_use_dependency(test_mc_rbdyn_urdf RBDyn)
# Adding a project configuration file (for MSVC only)
generate_msvc_dot_user_file(test_mc_rbdyn_urdf)
add_test(NAME "Test_mc_rbdyn_urdf" COMMAND test_mc_rbdyn_urdf)
