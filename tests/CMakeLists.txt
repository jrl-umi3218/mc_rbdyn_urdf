#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

enable_testing()

find_package(Boost REQUIRED COMPONENTS unit_test_framework system)

add_executable(test_mc_rbdyn_urdf test_mc_rbdyn_urdf.cpp)
target_link_libraries(test_mc_rbdyn_urdf PRIVATE mc_rbdyn_urdf Boost::unit_test_framework)
# Adding a project configuration file (for MSVC only)
generate_msvc_dot_user_file(test_mc_rbdyn_urdf)
add_test(NAME "Test_mc_rbdyn_urdf" COMMAND test_mc_rbdyn_urdf)
