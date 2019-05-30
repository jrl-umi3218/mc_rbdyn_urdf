#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

cmake_minimum_required(VERSION 3.1.3)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

include(cmake/base.cmake)
include(cmake/msvc-specific.cmake)

set(PROJECT_NAME mc_rbdyn_urdf)
set(PROJECT_DESCRIPTION "URDF parsing for the RBDyn library")
set(PROJECT_URL "https://github.com/jrl-umi3218/mc_rbdyn_urdf")
set(PROJECT_DEBUG_POSTFIX "_d")

set(DOXYGEN_USE_MATHJAX "YES")
set(CXX_DISABLE_WERROR True)
setup_project()
set(PROJECT_VERSION 1.1.0)

option(PYTHON_BINDING "Generate python binding." ON)
if(WIN32)
  set(PYTHON_BINDING_USER_INSTALL_DEFAULT ON)
else()
  set(PYTHON_BINDING_USER_INSTALL_DEFAULT OFF)
endif()
option(PYTHON_BINDING_USER_INSTALL "Install the Python bindings in user space" ${PYTHON_BINDING_USER_INSTALL_DEFAULT})
option(DISABLE_TESTS "Disable unit tests." OFF)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/CMakeModules)

add_project_dependency(RBDyn REQUIRED)
find_package(TinyXML2 REQUIRED)

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

setup_project_finalize()
setup_project_package_finalize()
