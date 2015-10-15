cmake_minimum_required(VERSION 2.8)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/boost.cmake)

SET(PROJECT_NAME mc_rbdyn_urdf_cpp)
SET(PROJECT_DESCRIPTION "...")
SET(PROJECT_URL "https://rob-dev.lirmm.fr/multi-contact/mc_rbdyn_urdf_cpp")

SET(DOXYGEN_USE_MATHJAX "YES")
SETUP_PROJECT()

option(UNIT_TESTS "Generate unit tests." ON)

if(NOT WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++0x -pedantic")
endif(NOT WIN32)

SEARCH_FOR_EIGEN()

ADD_REQUIRED_DEPENDENCY("SpaceVecAlg")
ADD_REQUIRED_DEPENDENCY("RBDyn")
ADD_REQUIRED_DEPENDENCY("tinyxml2")

add_subdirectory(src)

if(${UNIT_TESTS})
  add_subdirectory(tests)
endif()

PKG_CONFIG_APPEND_LIBS(mc_rbdyn_urdf)

SETUP_PROJECT_FINALIZE()
