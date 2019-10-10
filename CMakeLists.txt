#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.1.3)

set(CMAKE_CXX_STANDARD 11)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(PROJECT_NAME mc_rbdyn_urdf)
set(PROJECT_DESCRIPTION "URDF parsing for the RBDyn library")
set(PROJECT_URL "https://github.com/jrl-umi3218/mc_rbdyn_urdf")
set(PROJECT_DEBUG_POSTFIX "_d")
set(PROJECT_VERSION 1.1.0)
set(PROJECT_USE_CMAKE_EXPORT TRUE)

include(cmake/base.cmake)
include(cmake/cython/cython.cmake)
include(cmake/msvc-specific.cmake)

set(DOXYGEN_USE_MATHJAX "YES")
set(CXX_DISABLE_WERROR True)

project(mc_rbdyn_urdf CXX)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)

add_project_dependency(RBDyn REQUIRED)
find_package(tinyxml2 QUIET)
if(NOT ${tinyxml2_FOUND})
  find_package(mc_rbdyn_urdf_TinyXML2 REQUIRED)
  install(FILES "${PROJECT_SOURCE_DIR}/CMakeModules/Findmc_rbdyn_urdf_TinyXML2.cmake"
          DESTINATION "${CONFIG_INSTALL_DIR}")
   set(PACKAGE_EXTRA_MACROS "include(\${CMAKE_CURRENT_LIST_DIR}/Findmc_rbdyn_urdf_TinyXML2.cmake)")
else()
  add_project_dependency(tinyxml2 REQUIRED)
endif()

# Note: technically we don't need system but it is likely to be here and
# CMake <= 3.5.0 needs at least one component to define Boost::boost
add_project_dependency(Boost REQUIRED COMPONENTS system)

# For MSVC, set local environment variable to enable finding the built dll
# of the main library when launching ctest with RUN_TESTS
if(MSVC)
  set(CMAKE_MSVCIDE_RUN_PATH "\$(SolutionDir)/src/\$(Configuration)")
endif()

add_subdirectory(src)

if(${BUILD_TESTING})
  add_subdirectory(tests)
endif()

if(${PYTHON_BINDING})
 add_subdirectory(binding/python)
endif()
