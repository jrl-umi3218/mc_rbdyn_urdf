cmake_minimum_required(VERSION 2.8)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/eigen.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/msvc-specific.cmake)

SET(PROJECT_NAME mc_rbdyn_urdf_cpp)
SET(PROJECT_DESCRIPTION "...")
SET(PROJECT_URL "https://rob-dev.lirmm.fr/multi-contact/mc_rbdyn_urdf_cpp")
SET(PROJECT_DEBUG_POSTFIX "_d")

SET(DOXYGEN_USE_MATHJAX "YES")
SETUP_PROJECT()

option(UNIT_TESTS "Generate unit tests." ON)

if(NOT WIN32)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++0x -pedantic")
endif(NOT WIN32)

SEARCH_FOR_EIGEN()

ADD_REQUIRED_DEPENDENCY("SpaceVecAlg")
ADD_REQUIRED_DEPENDENCY("RBDyn" "RBDyn_d")
ADD_REQUIRED_DEPENDENCY("tinyxml2")

# For MSVC, set local environment variable to enable finding the built dll
# of the main library when launching ctest with RUN_TESTS
IF(MSVC)
  SET(CMAKE_MSVCIDE_RUN_PATH "\$(SolutionDir)/src/\$(Configuration)")
ENDIF(MSVC)

add_subdirectory(src)

if(${UNIT_TESTS})
  add_subdirectory(tests)
endif()

PKG_CONFIG_APPEND_LIBS(mc_rbdyn_urdf)

SETUP_PROJECT_FINALIZE()
