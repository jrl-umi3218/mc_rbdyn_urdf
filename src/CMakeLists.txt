#
# Copyright 2012-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

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
