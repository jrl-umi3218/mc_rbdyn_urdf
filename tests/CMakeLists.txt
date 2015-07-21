ENABLE_TESTING()

set(Boost_USE_STATIC_LIBS ON)
set(BOOST_COMPONENTS unit_test_framework)
SEARCH_FOR_BOOST()
include_directories(${Boost_INCLUDE_DIR})

add_executable(test_mc_rbdyn_urdf test_mc_rbdyn_urdf.cpp)
target_link_libraries(test_mc_rbdyn_urdf ${Boost_LIBRARIES} mc_rbdyn_urdf)
PKG_CONFIG_USE_DEPENDENCY(test_mc_rbdyn_urdf SpaceVecAlg)
PKG_CONFIG_USE_DEPENDENCY(test_mc_rbdyn_urdf RBDyn)
add_test(NAME "Test_mc_rbdyn_urdf" COMMAND test_mc_rbdyn_urdf)
