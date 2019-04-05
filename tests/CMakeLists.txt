#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

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
